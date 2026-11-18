package phantom.sim;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.AllianceLabel;
import phantom.core.tracking.Detection;
import phantom.core.tracking.DetectionFrame;
import phantom.core.tracking.DetectionSource;
import phantom.core.tracking.Measurement;
import phantom.core.tracking.MeasurementSource;

/**
 * Simulated robot-detecting camera that reports what a real pipeline reports: ANONYMOUS
 * detections (a position and a bumper-color guess, no robot id) of opponents AND our alliance
 * partners, plus false positives. Same camera model as {@link NoisySensor} (field of view, range,
 * Gaussian noise, latency, dropped detections, frame rate), with these additions:
 *
 * <pre>
 *  - color label: the true alliance (OPPONENT / PARTNER), flipped with probability colorErrorRate
 *  - occlusion:   a robot is not seen if the straight line from our camera (robot center) to its
 *                 center passes through another robot that is closer (see occluded())
 *  - clutter:     n ~ Poisson(clutterPerFrame) false detections per frame, uniform over the area
 *                 of the field-of-view sector:  angle = heading + (2 U1 - 1) * fov / 2,
 *                 r = range * sqrt(U2)  (sqrt makes it uniform per square meter, not per meter).
 *                 Points outside the field are dropped (a pipeline knows the field size).
 *                 Labelled OPPONENT with probability clutterOpponentShare, else PARTNER.
 *  - our own robot: with probability selfDetectionRate per frame the detector fires on our own
 *                 bumpers (position = our center + noise, label = PARTNER, possibly flipped).
 *                 The pipeline then drops EVERY detection closer than robotHalfSize to our own
 *                 center: nothing else can be inside our bumpers. A noisy self-detection that
 *                 lands outside that radius leaks through, like it would on a real robot.
 *  - persistent false positives (optional, off in the presets): persistentSpots fixed points,
 *                 placed at random on the field when the sensor is made, each with its own fixed
 *                 apparent color. While a spot is in view and not hidden behind a robot it is
 *                 "detected" with probability persistentRate per frame, like a field element or a
 *                 reflection the detector keeps mistaking for a robot.
 *  - the detections of one frame are shuffled: their order says nothing about identity.
 * </pre>
 *
 * <p>Every frame is delivered, even with no detections, so the tracker can count misses. Seeded,
 * so runs are repeatable.
 */
public final class AnonymousSensor implements DetectionSource {
  /**
   * @param camera field of view, range, noise, latency, dropout and frame rate (NoisySensor model)
   * @param colorErrorRate chance that a real robot's alliance label is wrong (0..1)
   * @param clutterPerFrame mean number of false detections per frame (Poisson), >= 0
   * @param clutterOpponentShare chance that a false detection is labelled OPPONENT (0..1)
   * @param selfDetectionRate chance per frame that our own bumpers are detected (0..1)
   * @param occlusion true = robots hidden behind other robots are not seen
   * @param persistentSpots number of fixed false-positive spots (0 = none)
   * @param persistentRate chance per frame that a spot in view is detected (0..1)
   */
  public record Config(
      NoisySensor.Config camera,
      double colorErrorRate,
      double clutterPerFrame,
      double clutterOpponentShare,
      double selfDetectionRate,
      boolean occlusion,
      int persistentSpots,
      double persistentRate) {

    public Config {
      if (!(colorErrorRate >= 0 && colorErrorRate <= 1)
          || !(clutterPerFrame >= 0 && Double.isFinite(clutterPerFrame))
          || !(clutterOpponentShare >= 0 && clutterOpponentShare <= 1)
          || !(selfDetectionRate >= 0 && selfDetectionRate <= 1)
          || persistentSpots < 0 || !(persistentRate >= 0 && persistentRate <= 1)) {
        throw new IllegalArgumentException("invalid AnonymousSensor.Config");
      }
    }

    /** No persistent false positives. */
    public Config(NoisySensor.Config camera, double colorErrorRate, double clutterPerFrame,
        double clutterOpponentShare, double selfDetectionRate, boolean occlusion) {
      this(camera, colorErrorRate, clutterPerFrame, clutterOpponentShare, selfDetectionRate,
          occlusion, 0, 0.0);
    }

    /**
     * The Realistic camera (two cameras, 140 deg, 0.10 m, 80 ms, 15% dropped, 8 m, 30 Hz) plus:
     * 5% wrong bumper color, 0.1 false detections per frame (3 per second), our own bumpers
     * detected in 10% of frames, occlusion on. The added numbers are GUESSES, like the camera
     * numbers, until the real pipeline is measured.
     */
    public static Config realistic() {
      return new Config(NoisySensor.Config.realistic(), 0.05, 0.10, 0.5, 0.10, true);
    }

    /**
     * The Bad camera (one 70 deg camera, 0.25 m, 150 ms, 35% dropped, 6 m, 15 Hz) plus: 15% wrong
     * bumper color, 0.5 false detections per frame (7.5 per second), our own bumpers detected in
     * 30% of frames, occlusion on. Guesses, as above.
     */
    public static Config bad() {
      return new Config(NoisySensor.Config.bad(), 0.15, 0.50, 0.5, 0.30, true);
    }

    public String name() {
      return camera.name();
    }

    public Config withOcclusion(boolean on) {
      return new Config(camera, colorErrorRate, clutterPerFrame, clutterOpponentShare,
          selfDetectionRate, on, persistentSpots, persistentRate);
    }

    public Config withColorError(double rate) {
      return new Config(camera, rate, clutterPerFrame, clutterOpponentShare, selfDetectionRate,
          occlusion, persistentSpots, persistentRate);
    }

    public Config withClutter(double perFrame, double opponentShare) {
      return new Config(camera, colorErrorRate, perFrame, opponentShare, selfDetectionRate,
          occlusion, persistentSpots, persistentRate);
    }

    public Config withSelfDetectionRate(double rate) {
      return new Config(camera, colorErrorRate, clutterPerFrame, clutterOpponentShare, rate,
          occlusion, persistentSpots, persistentRate);
    }

    /** Fixed false-positive spots (see the class comment). */
    public Config withPersistentFalsePositives(int spots, double ratePerFrame) {
      return new Config(camera, colorErrorRate, clutterPerFrame, clutterOpponentShare,
          selfDetectionRate, occlusion, spots, ratePerFrame);
    }
  }

  /** A frame plus, for each detection, who it really was: a body id, CLUTTER or SELF. */
  private record Pending(DetectionFrame frame, int[] sources, double deliverAt) {}

  private record Tagged(Detection detection, int source) {}

  /** Source of a false detection. */
  public static final int CLUTTER = -1;

  /** Source of a detection of our own bumpers. */
  public static final int SELF = -2;

  /** Source of a persistent false positive. */
  public static final int PHANTOM = -3;

  /** Persistent false-positive spots never sit closer than this to a field wall (m). */
  public static final double SPOT_MARGIN = 0.5;

  private final SimWorld world;
  private final Config config;
  private final Random random;
  private final List<Pending> pending = new ArrayList<>();
  private double nextSample = 0.0;

  private int frames = 0;
  private int robotDetections = 0;
  private int occludedRobots = 0;
  private int wrongLabels = 0;
  private int clutterDetections = 0;
  private int selfDetections = 0;
  private int removedNearSelf = 0;
  private int phantomDetections = 0;
  private final List<Vec2> spots = new ArrayList<>();
  private final List<AllianceLabel> spotColors = new ArrayList<>();

  public AnonymousSensor(SimWorld world, Config config, long seed) {
    this.world = world;
    this.config = config;
    this.random = new Random(seed);
    FieldConfig field = world.field();
    for (int i = 0; i < config.persistentSpots(); i++) {
      double x = SPOT_MARGIN + random.nextDouble() * (field.length() - 2 * SPOT_MARGIN);
      double y = SPOT_MARGIN + random.nextDouble() * (field.width() - 2 * SPOT_MARGIN);
      spots.add(new Vec2(x, y));
      spotColors.add(random.nextBoolean() ? AllianceLabel.OPPONENT : AllianceLabel.PARTNER);
    }
  }

  /** Where the persistent false positives are (empty unless configured). */
  public List<Vec2> persistentSpots() {
    return List.copyOf(spots);
  }

  public Config config() {
    return config;
  }

  @Override
  public List<DetectionFrame> poll(double nowSec) {
    List<DetectionFrame> out = new ArrayList<>();
    for (Pending p : due(nowSec)) {
      out.add(p.frame());
    }
    return out;
  }

  /**
   * The same camera with identity handed over, as the old model assumed: only detections of
   * opponents, each tagged with that opponent's id (partners, clutter and our own bumpers are
   * dropped). For measuring what occlusion alone costs. Use either this or {@link #poll}, not both.
   */
  public MeasurementSource knownIdView() {
    return nowSec -> {
      List<Measurement> out = new ArrayList<>();
      for (Pending p : due(nowSec)) {
        List<Detection> ds = p.frame().detections();
        for (int i = 0; i < ds.size(); i++) {
          if (isOpponent(p.sources()[i])) {
            out.add(new Measurement(p.sources()[i], ds.get(i).position(),
                p.frame().timestampSec()));
          }
        }
      }
      return out;
    };
  }

  private boolean isOpponent(int bodyId) {
    for (SimWorld.Opponent o : world.opponents()) {
      if (o.body().id() == bodyId) {
        return true;
      }
    }
    return false;
  }

  /** Captures a frame if one is due, then removes and returns every frame due for delivery. */
  private List<Pending> due(double nowSec) {
    if (nowSec + 1e-9 >= nextSample) {
      capture(nowSec);
      nextSample = nowSec + 1.0 / config.camera().rateHz();
    }
    List<Pending> out = new ArrayList<>();
    Iterator<Pending> it = pending.iterator();
    while (it.hasNext()) {
      Pending p = it.next();
      if (p.deliverAt() <= nowSec + 1e-9) {
        out.add(p);
        it.remove();
      }
    }
    return out;
  }

  private record Robot(int id, Vec2 position, AllianceLabel truth) {}

  private void capture(double nowSec) {
    NoisySensor.Config cam = config.camera();
    RobotBody self = world.self();
    Vec2 eye = self.position();
    double halfFov = Math.toRadians(cam.fovDeg()) / 2.0;
    FieldConfig field = world.field();

    List<Robot> robots = new ArrayList<>();
    for (SimWorld.Opponent o : world.opponents()) {
      robots.add(new Robot(o.body().id(), o.body().position(), AllianceLabel.OPPONENT));
    }
    for (SimWorld.Opponent p : world.partners()) {
      robots.add(new Robot(p.body().id(), p.body().position(), AllianceLabel.PARTNER));
    }

    List<Tagged> dets = new ArrayList<>();
    for (Robot r : robots) {
      Vec2 rel = r.position().minus(eye);
      if (rel.norm() > cam.rangeM()) {
        continue;
      }
      if (Math.abs(Angles.difference(rel.angle(), self.heading())) > halfFov) {
        continue;
      }
      if (config.occlusion()) {
        List<Vec2> others = new ArrayList<>();
        for (Robot o : robots) {
          if (o != r) {
            others.add(o.position());
          }
        }
        if (occluded(eye, r.position(), others, field.robotHalfSize())) {
          occludedRobots++;
          continue;
        }
      }
      if (random.nextDouble() < cam.dropout()) {
        continue;
      }
      dets.add(new Tagged(
          new Detection(addNoise(r.position(), cam.noiseStd()), label(r.truth())), r.id()));
      robotDetections++;
    }

    if (random.nextDouble() < config.selfDetectionRate()) {
      dets.add(new Tagged(
          new Detection(addNoise(eye, cam.noiseStd()), label(AllianceLabel.PARTNER)), SELF));
      selfDetections++;
    }

    int n = poisson(config.clutterPerFrame());
    for (int i = 0; i < n; i++) {
      double angle = self.heading() + (2.0 * random.nextDouble() - 1.0) * halfFov;
      double radius = cam.rangeM() * Math.sqrt(random.nextDouble());
      Vec2 p = eye.plus(Vec2.polar(radius, angle));
      AllianceLabel l =
          random.nextDouble() < config.clutterOpponentShare()
              ? AllianceLabel.OPPONENT
              : AllianceLabel.PARTNER;
      if (p.x() < 0 || p.x() > field.length() || p.y() < 0 || p.y() > field.width()) {
        continue; // outside the field: the pipeline throws it away
      }
      dets.add(new Tagged(new Detection(p, l), CLUTTER));
      clutterDetections++;
    }

    // Persistent false positives: same spot every time, seen like a robot would be.
    for (int i = 0; i < spots.size(); i++) {
      Vec2 spot = spots.get(i);
      Vec2 rel = spot.minus(eye);
      if (rel.norm() > cam.rangeM()
          || Math.abs(Angles.difference(rel.angle(), self.heading())) > halfFov) {
        continue;
      }
      if (config.occlusion()) {
        List<Vec2> blockers = new ArrayList<>();
        for (Robot r : robots) {
          blockers.add(r.position());
        }
        if (occluded(eye, spot, blockers, field.robotHalfSize())) {
          continue;
        }
      }
      if (random.nextDouble() < config.persistentRate()) {
        dets.add(new Tagged(
            new Detection(addNoise(spot, cam.noiseStd()), label(spotColors.get(i))), PHANTOM));
        phantomDetections++;
      }
    }

    // Our own robot: nothing real can be inside our bumpers.
    List<Tagged> kept = new ArrayList<>();
    for (Tagged d : dets) {
      if (d.detection().position().distanceTo(eye) < field.robotHalfSize()) {
        removedNearSelf++;
      } else {
        kept.add(d);
      }
    }
    Collections.shuffle(kept, random);
    List<Detection> frame = new ArrayList<>();
    int[] sources = new int[kept.size()];
    for (int i = 0; i < kept.size(); i++) {
      frame.add(kept.get(i).detection());
      sources[i] = kept.get(i).source();
    }
    frames++;
    pending.add(new Pending(new DetectionFrame(nowSec, frame), sources,
        nowSec + cam.latencySec()));
  }

  private Vec2 addNoise(Vec2 p, double std) {
    return p.plus(new Vec2(random.nextGaussian(), random.nextGaussian()).times(std));
  }

  private AllianceLabel label(AllianceLabel truth) {
    if (random.nextDouble() < config.colorErrorRate()) {
      wrongLabels++;
      return truth == AllianceLabel.OPPONENT ? AllianceLabel.PARTNER : AllianceLabel.OPPONENT;
    }
    return truth;
  }

  /** Poisson sample (Knuth: multiply uniforms until the product drops below e^-mean). */
  private int poisson(double mean) {
    if (mean <= 0) {
      return 0;
    }
    double limit = Math.exp(-mean);
    double product = random.nextDouble();
    int k = 0;
    while (product > limit) {
      k++;
      product *= random.nextDouble();
    }
    return k;
  }

  /**
   * True if the straight line from the camera to the target's center passes through another robot
   * that is closer to the camera (robots as circles of radius {@code blockerRadius}):
   *
   * <pre>
   *   D = |target - camera|,  u = (target - camera) / D
   *   for each other robot o:   s = (o - camera) . u       how far along the line
   *                             h = |u x (o - camera)|     how far from the line
   *   hidden if some o has  0 < s < D  and  h < blockerRadius
   * </pre>
   */
  public static boolean occluded(Vec2 camera, Vec2 target, List<Vec2> others, double blockerRadius) {
    Vec2 toTarget = target.minus(camera);
    double dist = toTarget.norm();
    Vec2 u = toTarget.normalized();
    if (u.equals(Vec2.ZERO)) {
      return false; // target on the camera: nothing can be in between
    }
    for (Vec2 o : others) {
      Vec2 rel = o.minus(camera);
      double s = rel.dot(u);
      double h = Math.abs(u.cross(rel));
      if (s > 0 && s < dist && h < blockerRadius) {
        return true;
      }
    }
    return false;
  }

  /** Frames captured so far. */
  public int frames() {
    return frames;
  }

  /** Detections of real robots (opponents and partners) produced so far. */
  public int robotDetections() {
    return robotDetections;
  }

  /** Times a robot inside the field of view was hidden behind another robot. */
  public int occludedRobots() {
    return occludedRobots;
  }

  /** Real-robot or self detections whose alliance label was flipped. */
  public int wrongLabels() {
    return wrongLabels;
  }

  /** False detections that made it into a frame (before the own-robot filter). */
  public int clutterDetections() {
    return clutterDetections;
  }

  /** Frames in which the detector fired on our own bumpers. */
  public int selfDetections() {
    return selfDetections;
  }

  /** Detections dropped because they were inside our own bumpers. */
  public int removedNearSelf() {
    return removedNearSelf;
  }

  /** Detections of persistent false-positive spots produced so far. */
  public int phantomDetections() {
    return phantomDetections;
  }
}
