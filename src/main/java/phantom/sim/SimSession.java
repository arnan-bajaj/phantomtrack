package phantom.sim;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.UnaryOperator;
import phantom.core.control.ChassisCommand;
import phantom.core.control.DriverBlend;
import phantom.core.control.HolonomicController;
import phantom.core.field.FieldConfig;
import phantom.core.manager.DefenseAssist;
import phantom.core.modes.CutoffMode;
import phantom.core.modes.DefenseTarget;
import phantom.core.modes.InterposeMode;
import phantom.core.modes.LineMirrorMode;
import phantom.core.modes.ShadowMode;
import phantom.core.modes.ZoneMode;
import phantom.core.geometry.Vec2;
import phantom.core.control.RulesGuard;
import phantom.core.rules.ContactDetector;
import phantom.core.tracking.Hungarian;
import phantom.core.tracking.KalmanOpponentTracker;
import phantom.core.tracking.MultiTargetTracker;
import phantom.core.tracking.OpponentTracker;
import phantom.core.tracking.TrackedRobot;

/**
 * One running simulation: the world, the defense assist wired to a tracker, and the metrics. Used
 * by both the window and headless mode so they behave identically.
 */
public final class SimSession {
  public static final double DT = 0.02; // 50 Hz, same as the robot loop

  /** How the defense code learns where opponents are. */
  public enum Sensor {
    /** Exact ground truth, no delay. */
    PERFECT(null, null, 0.05),
    /** Simulated camera + Kalman filter, plausible numbers. */
    REALISTIC(NoisySensor.Config.realistic(), AnonymousSensor.Config.realistic(), 0.4),
    /** Simulated camera + Kalman filter, struggling pipeline. */
    BAD(NoisySensor.Config.bad(), AnonymousSensor.Config.bad(), 0.4);

    final NoisySensor.Config config;
    final AnonymousSensor.Config anonymous;
    /**
     * Typical tracking error (m), used to size the contact detector's vision ramp. Realistic is
     * the measured 0.4 m; Bad (0.9 m measured) is capped at 0.4 m because a wider ramp would call
     * contact at almost 1 m apart. Perfect gets 0.05 m.
     */
    final double trackingErrorM;

    Sensor(NoisySensor.Config config, AnonymousSensor.Config anonymous, double trackingErrorM) {
      this.config = config;
      this.anonymous = anonymous;
      this.trackingErrorM = trackingErrorM;
    }

    public String label() {
      return config == null ? "Perfect" : config.name();
    }

    public Sensor next() {
      return values()[(ordinal() + 1) % values().length];
    }
  }

  /** Whether camera sightings come with the opponent's identity. Ignored for PERFECT. */
  public enum Tracking {
    /** The old assumption: every sighting says which opponent it is. NoisySensor + Kalman. */
    KNOWN_ID("Known-ID"),
    /**
     * Known ids, but the new camera's physics: AnonymousSensor's occlusion, with ids handed over
     * for opponents (no clutter, no self-detections, partners ignored) + Kalman. Separates the
     * cost of occlusion from the cost of anonymity.
     */
    KNOWN_ID_OCCLUSION("Known+occ"),
    /** What a real camera gives: anonymous detections. AnonymousSensor + MultiTargetTracker. */
    ANONYMOUS("Anonymous");

    private final String label;

    Tracking(String label) {
      this.label = label;
    }

    public String label() {
      return label;
    }

    public Tracking next() {
      return values()[(ordinal() + 1) % values().length];
    }
  }

  /** Variations of the anonymous pipeline, for experiments. */
  public enum Variant {
    /** MultiTargetTracker defaults. */
    STANDARD(c -> c),
    /** Re-acquisition (id handoff) switched off. */
    NO_HANDOFF(MultiTargetTracker.Config::withoutHandoff);

    final UnaryOperator<MultiTargetTracker.Config> tuning;

    Variant(UnaryOperator<MultiTargetTracker.Config> tuning) {
      this.tuning = tuning;
    }
  }

  /** Track ids of the multi-target tracker start here, far from every body id in the sim. */
  public static final int FIRST_TRACK_ID = 1000;

  /** Focus id meaning "locked, but the driver has not pointed at anyone yet": matches no track. */
  public static final int NOT_DESIGNATED = Integer.MAX_VALUE;

  /**
   * Simulated driver for "locked on one opponent" with anonymous tracks: time to notice that the
   * assist lost its target or is on the wrong robot, and to point it at the right one again.
   */
  public static final double DRIVER_REACTION_SEC = 0.5;

  private final SimWorld world;
  private final Sensor sensor;
  private final Tracking tracking;
  private final MultiTargetTracker multiTracker; // null unless anonymous
  private final AnonymousSensor anonymousSensor; // null unless anonymous
  private final DefenseAssist assist;
  private final DriveTelemetrySim driveTelemetry;
  private final Metrics metrics = new Metrics();
  private ChassisCommand lastCommand = ChassisCommand.STOP;

  // Simulated driver (anonymous tracking, locked focus only)
  private int lockedOpponent = -1; // index into world.opponents(), -1 = not locked
  private boolean designated = false;
  private double driverProblemSince = Double.NaN;
  private int redesignations = 0;

  public SimSession(SimWorld world) {
    this(world, Sensor.PERFECT, 1L);
  }

  /**
   * @param seed random seed for the simulated camera (same seed = same run)
   */
  public SimSession(SimWorld world, Sensor sensor, long seed) {
    this(world, sensor, seed, Tracking.KNOWN_ID, Variant.STANDARD);
  }

  /**
   * @param seed random seed for the simulated camera (same seed = same run)
   * @param tracking known-id sightings (old model) or anonymous detections; ignored for PERFECT
   * @param variant anonymous pipeline variation (ignored for known-id)
   */
  public SimSession(SimWorld world, Sensor sensor, long seed, Tracking tracking, Variant variant) {
    this(world, sensor, seed, tracking, variant.tuning);
  }

  /**
   * @param tuning changes to the MultiTargetTracker defaults (anonymous tracking only)
   */
  public SimSession(SimWorld world, Sensor sensor, long seed, Tracking tracking,
      UnaryOperator<MultiTargetTracker.Config> tuning) {
    this(world, sensor, seed, sensor.config == null ? Tracking.KNOWN_ID : tracking,
        Pipeline.build(world, sensor, seed, sensor.config == null ? Tracking.KNOWN_ID : tracking,
            tuning));
  }

  /**
   * For tests: drive the assist with any tracker (e.g. a scripted one).
   *
   * @param sensor only used as a label (and to draw estimates when not PERFECT)
   * @param tracking ANONYMOUS makes the metrics and the simulated driver ignore track ids
   */
  SimSession(SimWorld world, Sensor sensor, Tracking tracking, OpponentTracker tracker) {
    this(world, sensor, 1L, tracking, new Pipeline(tracker, null, null));
  }

  /**
   * For experiments: a camera other than the presets. Anonymous tracking uses MultiTargetTracker
   * defaults changed by {@code tuning}; known-id tracking uses the same camera model with ids
   * handed over (no occlusion, like the old NoisySensor) and the usual Kalman tracker.
   *
   * @param label only used as a label (and to draw estimates)
   */
  static SimSession withCamera(SimWorld world, Sensor label, AnonymousSensor.Config camera,
      long seed, Tracking tracking, UnaryOperator<MultiTargetTracker.Config> tuning) {
    if (tracking == Tracking.ANONYMOUS) {
      AnonymousSensor cam = new AnonymousSensor(world, camera, seed);
      MultiTargetTracker mtt = new MultiTargetTracker(cam, tuning.apply(
          MultiTargetTracker.Config.defaults(camera.camera().noiseStd(), camera.colorErrorRate())
              .withFirstTrackId(FIRST_TRACK_ID)));
      return new SimSession(world, label, seed, tracking, new Pipeline(mtt, mtt, cam));
    }
    return new SimSession(world, label, seed, Tracking.KNOWN_ID, new Pipeline(
        KalmanOpponentTracker.defaults(new NoisySensor(world, camera.camera(), seed),
            camera.camera().noiseStd()), null, null));
  }

  private SimSession(
      SimWorld world, Sensor sensor, long seed, Tracking tracking, Pipeline pipeline) {
    this.world = world;
    this.sensor = sensor;
    this.tracking = tracking;
    this.multiTracker = pipeline.multiTracker();
    this.anonymousSensor = pipeline.anonymousSensor();
    // Drive sensors for contact detection: noisy whenever the camera is, on their own seed.
    this.driveTelemetry =
        new DriveTelemetrySim(
            world.self(),
            DriveMotorModel.krakenX60(),
            sensor.config == null
                ? DriveTelemetrySim.Config.noiseless()
                : DriveTelemetrySim.Config.realistic(),
            31L * seed + 7L);
    this.assist =
        new DefenseAssist(
            List.of(
                ShadowMode.defaults(),
                InterposeMode.defaults(),
                ZoneMode.defaults(),
                LineMirrorMode.defaults(),
                CutoffMode.defaults()),
            HolonomicController.defaults(),
            DriverBlend.defaults(),
            pipeline.tracker(),
            world.field());
    // The detector's vision ramp is a per-camera calibration: as wide as the tracking error.
    this.assist.setContactDetectorConfig(
        ContactDetector.Config.placeholder().withVisionError(sensor.trackingErrorM));
    // Pin margin from our robot's own limits: the same rule as on the real robot.
    this.assist.setDriveLimits(
        world.self().maxSpeed(), world.self().maxAccel(), RulesGuard.SIM_LATENCY_SECONDS);
  }

  /** Camera + tracker for one session. */
  private record Pipeline(
      OpponentTracker tracker, MultiTargetTracker multiTracker, AnonymousSensor anonymousSensor) {

    static Pipeline build(SimWorld world, Sensor sensor, long seed, Tracking tracking,
        UnaryOperator<MultiTargetTracker.Config> tuning) {
      if (sensor.config == null) {
        return new Pipeline(new PerfectTracker(world), null, null);
      }
      if (tracking == Tracking.KNOWN_ID) {
        return new Pipeline(KalmanOpponentTracker.defaults(
            new NoisySensor(world, sensor.config, seed), sensor.config.noiseStd()), null, null);
      }
      if (tracking == Tracking.KNOWN_ID_OCCLUSION) {
        AnonymousSensor camera = new AnonymousSensor(world,
            sensor.anonymous.withClutter(0.0, 0.5).withSelfDetectionRate(0.0), seed);
        return new Pipeline(KalmanOpponentTracker.defaults(
            camera.knownIdView(), sensor.anonymous.camera().noiseStd()), null, camera);
      }
      AnonymousSensor camera = new AnonymousSensor(world, sensor.anonymous, seed);
      MultiTargetTracker.Config c =
          MultiTargetTracker.Config.defaults(
                  sensor.anonymous.camera().noiseStd(), sensor.anonymous.colorErrorRate())
              .withFirstTrackId(FIRST_TRACK_ID);
      MultiTargetTracker mtt = new MultiTargetTracker(camera, tuning.apply(c));
      return new Pipeline(mtt, mtt, camera);
    }
  }

  public static SimSession standard() {
    return standard(Scenarios.OpponentKind.CYCLER);
  }

  public static SimSession standard(Scenarios.OpponentKind kind) {
    return new SimSession(Scenarios.three(FieldConfig.placeholder(), kind));
  }

  public static SimSession standard(Scenarios.OpponentKind kind, Sensor sensor, long seed) {
    return standard(kind, sensor, seed, Scenarios.Layout.OPEN);
  }

  public static SimSession standard(
      Scenarios.OpponentKind kind, Sensor sensor, long seed, Scenarios.Layout layout) {
    return new SimSession(Scenarios.three(layout.field(), kind), sensor, seed);
  }

  /** Full version: optional alliance partners and anonymous tracking. */
  public static SimSession standard(Scenarios.OpponentKind kind, Sensor sensor, long seed,
      Scenarios.Layout layout, boolean partners, Tracking tracking, Variant variant) {
    SimWorld w = Scenarios.three(layout.field(), kind, Scenarios.DEFENDER_START, partners);
    return new SimSession(w, sensor, seed, tracking, variant);
  }

  public Sensor sensor() {
    return sensor;
  }

  /** KNOWN_ID for the perfect sensor and the old camera model; ANONYMOUS for the new one. */
  public Tracking tracking() {
    return tracking;
  }

  /** The multi-target tracker, or null when tracking is not anonymous. */
  public MultiTargetTracker multiTracker() {
    return multiTracker;
  }

  /** The anonymous camera, or null when tracking is not anonymous. */
  public AnonymousSensor anonymousSensor() {
    return anonymousSensor;
  }

  /**
   * Guard one opponent (man-to-man). With known ids that is simply its id. With anonymous tracks
   * the ids mean nothing in advance, so a simulated driver points the assist at the track that is
   * on that opponent, and points it again {@link #DRIVER_REACTION_SEC} after it loses the target
   * or ends up on the wrong robot (see {@link #redesignations()}).
   *
   * @param opponentIndex index into world().opponents()
   */
  public void lockOn(int opponentIndex) {
    if (tracking == Tracking.ANONYMOUS) {
      lockedOpponent = opponentIndex;
      designated = false;
      assist.setFocusOpponentId(NOT_DESIGNATED);
    } else {
      assist.setFocusOpponentId(world.opponents().get(opponentIndex).body().id());
    }
  }

  /** Times the simulated driver had to point the assist at its target again (not the first). */
  public int redesignations() {
    return redesignations;
  }

  /** Opponent index the simulated driver keeps the assist on, or -1 (not locked / known ids). */
  public int lockedOpponent() {
    return lockedOpponent;
  }

  /** Let auto focus pick (stops the simulated driver). */
  public void autoFocus() {
    lockedOpponent = -1;
    assist.setFocusOpponentId(-1);
  }

  /** Run one 20 ms loop: assist decides, world moves, metrics update. */
  public void step(ChassisCommand driver, boolean assistEngaged) {
    simulatedDriver();
    RobotBody self = world.self();
    assist.setDriveTelemetry(driveTelemetry.sample(world.timeSec()));
    ChassisCommand cmd =
        assist.update(world.timeSec(), self.pose(), self.velocity(), driver, assistEngaged);
    lastCommand = cmd;
    world.step(cmd, DT);
    metrics.record(this);
  }

  /** The command sent to our robot on the last step. */
  public ChassisCommand lastCommand() {
    return lastCommand;
  }

  /**
   * The driver's part of "locked on opponent k" with anonymous tracks. Uses what the driver can
   * see: which track the HUD shows on that robot (the metrics' match from the last step).
   *
   * <pre>
   *   onTarget = track matched to opponent k right now (-1 if none)
   *   first time onTarget exists: point the assist at it
   *   afterwards, if focus != onTarget and onTarget exists for DRIVER_REACTION_SEC in a row:
   *     point the assist at onTarget again (a re-designation)
   * </pre>
   */
  private void simulatedDriver() {
    if (lockedOpponent < 0) {
      return;
    }
    int onTarget = metrics.trackOnOpponent(lockedOpponent);
    if (!designated) {
      if (onTarget >= 0) {
        assist.setFocusOpponentId(onTarget);
        designated = true;
      }
      return;
    }
    if (onTarget < 0 || assist.focusOpponentId() == onTarget) {
      driverProblemSince = Double.NaN;
      return;
    }
    double now = world.timeSec();
    if (Double.isNaN(driverProblemSince)) {
      driverProblemSince = now;
    }
    if (now - driverProblemSince >= DRIVER_REACTION_SEC - 1e-9) {
      assist.setFocusOpponentId(onTarget);
      redesignations++;
      driverProblemSince = Double.NaN;
    }
  }

  public SimWorld world() {
    return world;
  }

  /** The simulated drive sensors feeding the contact detector. */
  public DriveTelemetrySim driveTelemetry() {
    return driveTelemetry;
  }

  public DefenseAssist assist() {
    return assist;
  }

  public Metrics metrics() {
    return metrics;
  }

  /** Same as below with no memory of the last step. */
  public static int[] matchTracks(List<TrackedRobot> tracks, double now, List<Vec2> truth,
      double radius) {
    return matchTracks(tracks, now, truth, radius, null);
  }

  /**
   * Matches tracks to real robots without using ids, the CLEAR-MOT way (Bernardin and
   * Stiefelhagen 2008, the standard multi-object tracking benchmark):
   *
   * <pre>
   *   d(track i, robot j) = |track_i.positionAt(now) - robot_j|
   *   1. keep last step's pair (robot j, track id) if that track is still reported and d <= radius
   *   2. the remaining tracks and robots: Hungarian on cost = d, pairs with d > radius not allowed
   *      (the most pairs first, then the smallest total distance)
   * </pre>
   *
   * Step 1 is what keeps a stale track drifting past a robot from "stealing" it from the track
   * that has been following it.
   *
   * @param truth true robot positions (opponents first, then partners)
   * @param previousTrackOfTruth track id matched to each robot on the last step (-1 = none), or
   *     null for no memory
   * @return for each track, the index into {@code truth} it is matched to, or -1
   */
  public static int[] matchTracks(List<TrackedRobot> tracks, double now, List<Vec2> truth,
      double radius, int[] previousTrackOfTruth) {
    int nTracks = tracks.size();
    int nRobots = truth.size();
    int[] out = new int[nTracks];
    java.util.Arrays.fill(out, -1);
    if (nTracks == 0 || nRobots == 0) {
      return out;
    }
    double[][] dist = new double[nTracks][nRobots];
    for (int i = 0; i < nTracks; i++) {
      Vec2 p = tracks.get(i).positionAt(now);
      for (int j = 0; j < nRobots; j++) {
        dist[i][j] = p.isFinite() ? p.distanceTo(truth.get(j)) : Double.POSITIVE_INFINITY;
      }
    }
    boolean[] robotUsed = new boolean[nRobots];
    if (previousTrackOfTruth != null) {
      for (int j = 0; j < nRobots && j < previousTrackOfTruth.length; j++) {
        for (int i = 0; i < nTracks; i++) {
          if (out[i] < 0 && previousTrackOfTruth[j] >= 0
              && tracks.get(i).id() == previousTrackOfTruth[j] && dist[i][j] <= radius) {
            out[i] = j;
            robotUsed[j] = true;
            break;
          }
        }
      }
    }
    List<Integer> freeTracks = new java.util.ArrayList<>();
    List<Integer> freeRobots = new java.util.ArrayList<>();
    for (int i = 0; i < nTracks; i++) {
      if (out[i] < 0) {
        freeTracks.add(i);
      }
    }
    for (int j = 0; j < nRobots; j++) {
      if (!robotUsed[j]) {
        freeRobots.add(j);
      }
    }
    if (freeTracks.isEmpty() || freeRobots.isEmpty()) {
      return out;
    }
    double[][] cost = new double[freeTracks.size()][freeRobots.size()];
    for (int a = 0; a < freeTracks.size(); a++) {
      for (int b = 0; b < freeRobots.size(); b++) {
        double d = dist[freeTracks.get(a)][freeRobots.get(b)];
        cost[a][b] = d <= radius ? d : MultiTargetTracker.FORBIDDEN;
      }
    }
    int[] assigned = Hungarian.solve(cost);
    for (int a = 0; a < assigned.length; a++) {
      if (assigned[a] >= 0 && cost[a][assigned[a]] <= radius) {
        out[freeTracks.get(a)] = freeRobots.get(assigned[a]);
      }
    }
    return out;
  }

  /** Running statistics about how well the defense is doing. */
  public static final class Metrics {
    /** A track within this distance of a robot (center to center) counts as tracking it. */
    public static final double MATCH_RADIUS = 1.5;

    /**
     * A track whose last sighting is at most this old is "fresh" (seen in about the latest frame:
     * latency 0.08-0.15 s plus one frame); older ones are coasting on a prediction.
     */
    public static final double FRESH_AGE_SEC = 0.3;

    // Identity-free tracking scores (same scorer for every tracker)
    private int steps = 0;
    private int[] coveredSteps = new int[0];
    private int[] lastMatchedTrack = new int[0];
    private int[] trackOnOpponent = new int[0];
    private int idSwitches = 0;
    private double squaredErrorSum = 0.0;
    private int matchedPairs = 0;
    private double falseTrackSeconds = 0.0;
    private double freshFalseTrackSeconds = 0.0;
    private double partnerAsOpponentSeconds = 0.0;
    private double freshPartnerAsOpponentSeconds = 0.0;
    private int[] previousTrackOfTruth = new int[0];
    private final Map<Integer, Vec2> truthOfTrack = new HashMap<>();
    private final Map<Integer, Boolean> trackOnPartner = new HashMap<>();
    private double guardingNothingSeconds = 0.0;
    private double guardingPartnerSeconds = 0.0;

    private double engagedSeconds = 0.0;
    private double guardingSeconds = 0.0; // engaged AND reacting to a specific opponent
    private double goalSideSeconds = 0.0;
    private double trackingErrorSum = 0.0;
    private int engagedSamples = 0;
    private double estimateErrorSum = 0.0;
    private int estimateSamples = 0;
    private int opponentSamples = 0;
    private int lastGuardedId = -1;
    private int targetSwitches = 0;
    private double falsePositiveSeconds = 0.0;
    private double missedSeconds = 0.0;
    private double pressingSeconds = 0.0;
    private boolean previousNear = false;
    private boolean previousPressing = false;
    private boolean havePreviousTruth = false;

    void record(SimSession s) {
      recordIdentityFree(s);
      if (s.tracking != Tracking.ANONYMOUS) {
        recordEstimates(s); // needs real ids
      }
      recordContactSensing(s);
      DefenseAssist.State st = s.assist.lastState();
      if (st != DefenseAssist.State.ENGAGED && st != DefenseAssist.State.BACKING_OFF) {
        return;
      }
      Optional<DefenseTarget> target = s.assist.lastTarget();
      if (target.isEmpty()) {
        return;
      }
      engagedSeconds += DT;
      engagedSamples++;
      int guarded = target.get().opponentId();
      if (guarded >= 0) {
        if (lastGuardedId >= 0 && guarded != lastGuardedId) {
          targetSwitches++;
        }
        lastGuardedId = guarded;
      }
      var selfPos = s.world.self().position();
      trackingErrorSum += selfPos.distanceTo(target.get().position());

      // Goal-side = we are closer to the goal than the opponent we are guarding.
      if (target.get().opponentId() < 0) {
        return; // e.g. Zone waiting at its anchor: nobody to be goal-side of
      }
      guardingSeconds += DT;
      Boolean onPartner = trackOnPartner.get(target.get().opponentId());
      if (onPartner == null) {
        guardingNothingSeconds += DT; // the target's track is on no robot (stale or phantom)
      } else if (onPartner) {
        guardingPartnerSeconds += DT; // the target's track is really one of our partners
      }
      var goal = s.world.field().defendedGoal();
      if (s.tracking == Tracking.ANONYMOUS) {
        Vec2 onRobot = truthOfTrack.get(target.get().opponentId()); // the robot that track is on
        if (onRobot != null && selfPos.distanceTo(goal) < onRobot.distanceTo(goal)) {
          goalSideSeconds += DT;
        }
        return;
      }
      for (TrackedRobot r : s.world.truth()) {
        if (r.id() == target.get().opponentId()
            && selfPos.distanceTo(goal) < r.position().distanceTo(goal)) {
          goalSideSeconds += DT;
        }
      }
    }

    /**
     * Scores the tracks the defense used this step against the truth (see {@link #matchTracks}):
     * coverage, position error, id switches, false tracks, partners reported as opponents.
     */
    private void recordIdentityFree(SimSession s) {
      double now = s.world.timeSec();
      List<TrackedRobot> tracks = s.assist.lastOpponents();
      List<TrackedRobot> opps = s.world.truth();
      List<TrackedRobot> partners = s.world.partnerTruth();
      int nOpp = opps.size();
      if (coveredSteps.length != nOpp) {
        coveredSteps = new int[nOpp];
        lastMatchedTrack = new int[nOpp];
        java.util.Arrays.fill(lastMatchedTrack, -1);
        trackOnOpponent = new int[nOpp];
      }
      java.util.Arrays.fill(trackOnOpponent, -1);
      truthOfTrack.clear();
      trackOnPartner.clear();
      List<Vec2> truth = new java.util.ArrayList<>();
      for (TrackedRobot r : opps) {
        truth.add(r.position());
      }
      for (TrackedRobot r : partners) {
        truth.add(r.position());
      }
      if (previousTrackOfTruth.length != truth.size()) {
        previousTrackOfTruth = new int[truth.size()];
        java.util.Arrays.fill(previousTrackOfTruth, -1);
      }
      int[] match = matchTracks(tracks, now, truth, MATCH_RADIUS, previousTrackOfTruth);
      java.util.Arrays.fill(previousTrackOfTruth, -1);
      steps++;
      for (int i = 0; i < tracks.size(); i++) {
        int j = match[i];
        int id = tracks.get(i).id();
        boolean fresh = now - tracks.get(i).timestampSec() <= FRESH_AGE_SEC;
        if (j < 0) {
          falseTrackSeconds += DT;
          freshFalseTrackSeconds += fresh ? DT : 0.0;
          continue;
        }
        previousTrackOfTruth[j] = id;
        truthOfTrack.put(id, truth.get(j));
        trackOnPartner.put(id, j >= nOpp);
        if (j >= nOpp) {
          partnerAsOpponentSeconds += DT;
          freshPartnerAsOpponentSeconds += fresh ? DT : 0.0;
          continue;
        }
        coveredSteps[j]++;
        trackOnOpponent[j] = id;
        double e = tracks.get(i).positionAt(now).distanceTo(truth.get(j));
        squaredErrorSum += e * e;
        matchedPairs++;
        if (lastMatchedTrack[j] >= 0 && lastMatchedTrack[j] != id) {
          idSwitches++;
        }
        lastMatchedTrack[j] = id;
      }
    }

    /** Track id matched to opponent k on the last step, or -1. */
    int trackOnOpponent(int k) {
      return k < trackOnOpponent.length ? trackOnOpponent[k] : -1;
    }

    /** Root-mean-square distance between opponents and the tracks matched to them (m). */
    public double positionRmse() {
      return matchedPairs == 0 ? Double.NaN : Math.sqrt(squaredErrorSum / matchedPairs);
    }

    /** Times a real opponent's matched track id changed (CLEAR-MOT "ID switches"). */
    public int idSwitches() {
      return idSwitches;
    }

    /** Fraction of the time opponent k had a reported track on it (0..1). */
    public double coverage(int k) {
      return steps == 0 || k >= coveredSteps.length ? Double.NaN : (double) coveredSteps[k] / steps;
    }

    /** Mean of {@link #coverage(int)} over all opponents (0..1). */
    public double meanCoverage() {
      if (coveredSteps.length == 0) {
        return Double.NaN;
      }
      double sum = 0.0;
      for (int k = 0; k < coveredSteps.length; k++) {
        sum += coverage(k);
      }
      return sum / coveredSteps.length;
    }

    /** Track-seconds of reported opponents that were not on any real robot. */
    public double falseTrackSeconds() {
      return falseTrackSeconds;
    }

    /** Part of {@link #falseTrackSeconds()} from fresh tracks (phantoms, not stale coasting). */
    public double freshFalseTrackSeconds() {
      return freshFalseTrackSeconds;
    }

    /** Track-seconds of reported opponents that were really one of our partners. */
    public double partnerAsOpponentSeconds() {
      return partnerAsOpponentSeconds;
    }

    /** Part of {@link #partnerAsOpponentSeconds()} from fresh tracks. */
    public double freshPartnerAsOpponentSeconds() {
      return freshPartnerAsOpponentSeconds;
    }

    /**
     * Seconds the engaged assist was reacting to a track that was on no robot (a stale track that
     * drifted off, or a phantom).
     */
    public double guardingNothingSeconds() {
      return guardingNothingSeconds;
    }

    /** Seconds the engaged assist was reacting to a track that was really one of our partners. */
    public double guardingPartnerSeconds() {
      return guardingPartnerSeconds;
    }

    /** Simulated seconds scored so far. */
    public double seconds() {
      return steps * DT;
    }

    /**
     * Scores the pin guard's contact sensing against ground truth. The guard decides at the start
     * of a step from sensor data describing the end of the previous step, so it is compared with
     * the truth recorded at the end of the previous step.
     *
     * <pre>
     *   false positive: guard counting contact while truly no opponent is within 0.05 m
     *   missed:         guard NOT counting contact while we truly press an opponent
     *                   (within 0.05 m and driving into it, what the referee counts)
     * </pre>
     */
    private void recordContactSensing(SimSession s) {
      if (!s.assist.rulesGuardEnabled()) {
        return;
      }
      boolean sensed = s.assist.guardSensesContact();
      if (havePreviousTruth) {
        if (sensed && !previousNear) {
          falsePositiveSeconds += DT;
        }
        if (previousPressing) {
          pressingSeconds += DT;
          if (!sensed) {
            missedSeconds += DT;
          }
        }
      }
      previousNear = !s.world.selfNearContactNow().isEmpty();
      previousPressing = !s.world.selfPressingNow().isEmpty();
      havePreviousTruth = true;
    }

    /** Seconds the guard counted contact while no opponent was touching us. */
    public double falsePositiveSeconds() {
      return falsePositiveSeconds;
    }

    /** Seconds we truly pressed an opponent while the guard did not count contact. */
    public double missedSeconds() {
      return missedSeconds;
    }

    /** Seconds we truly pressed an opponent (referee's definition). */
    public double pressingSeconds() {
      return pressingSeconds;
    }

    /** How wrong the robot's belief is: |estimated position now - true position|, per track. */
    private void recordEstimates(SimSession s) {
      double now = s.world.timeSec();
      for (TrackedRobot truth : s.world.truth()) {
        opponentSamples++;
        for (TrackedRobot est : s.assist.lastOpponents()) {
          if (est.id() == truth.id()) {
            estimateErrorSum += est.positionAt(now).distanceTo(truth.position());
            estimateSamples++;
          }
        }
      }
    }

    /** Mean distance between estimated and true opponent positions (m), over tracked samples. */
    public double meanEstimateError() {
      return estimateSamples == 0 ? Double.NaN : estimateErrorSum / estimateSamples;
    }

    /** Fraction of the time each opponent was being tracked at all (0..1). */
    public double trackCoverage() {
      return opponentSamples == 0 ? Double.NaN : (double) estimateSamples / opponentSamples;
    }

    /** Times the guarded opponent changed while engaged. */
    public int targetSwitches() {
      return targetSwitches;
    }

    public double engagedSeconds() {
      return engagedSeconds;
    }

    /** Fraction of guarding time spent goal-side of the guarded opponent (0..1). */
    public double goalSideFraction() {
      return guardingSeconds == 0 ? Double.NaN : goalSideSeconds / guardingSeconds;
    }

    /** Average distance between where we are and where the mode wants us (m). */
    public double meanTrackingError() {
      return engagedSamples == 0 ? Double.NaN : trackingErrorSum / engagedSamples;
    }
  }
}
