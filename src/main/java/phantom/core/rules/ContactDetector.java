package phantom.core.rules;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Decides whether we are touching an opponent by fusing four cues. Three come from our own
 * drivetrain and work with no camera at all; the fourth is the camera.
 *
 * <pre>
 *   ramp(x; a, b) = clamp((x - a) / (b - a), 0, 1)        (0 at a, 1 at b; a may be above b)
 *
 *   u         = unit(v_cmd),   pushing = |v_cmd| >= pushMinSpeed
 *   progress  = (v_meas . u) / |v_cmd|                    (1 = moving as asked, 0 = not moving)
 *   speedingUp = ramp(a_meas . u; 0, freeAccel)           (1 = accelerating freely: not blocked)
 *
 *   (a) stall   = ramp(progress; stallProgressNone, stallProgressFull) * (1 - speedingUp)
 *   (b) current = ramp(I; currentLow, currentHigh)                     * (1 - speedingUp)
 *                 only while v_meas . u <= |v_cmd| (faster than asked = braking current)
 *                 (if the current is not measured, current = stall)
 *   (c) decel   = ramp(-(a_meas . u); decelLow, decelHigh)   only while v_meas . u <= |v_cmd|
 *                 (we did not ask to slow down); after a hit it fades to 0 over decelHoldSec
 *   (d) vision_i = ramp(gap_i; visionGapFar, visionGapNear),  gap_i = |p_i - p_self| - 2 * halfSize
 *   (a), (b), (c) are 0 unless pushing.
 *
 *   physical   = wStall * stall + wCurrent * current + wDecel * decel
 *   explains_i = pushing and gap_i <= attributeGap and u . unit(p_i - p_self) >= attributeCos
 *                (opponent i is near and ahead: it can be the thing we are pushing on)
 *
 *   L_i       = bias + wVision * vision_i + (explains_i ? physical : 0)      per tracked opponent
 *   L_unknown = bias + max_j (wVision * vision_j) + (no identified opponent explains it ? physical : 0)
 *               j over tracks WITHOUT a usable id (id < 0, or an id repeated in the same list);
 *               max over nothing = 0, which is the case with a normal tracker
 *   L_blocked = bias + physical
 *   p_raw     = 1 / (1 + exp(-L))
 *
 *   smoothing:  p += (p_raw - p) * dt / (tau + dt),  tau = tauRise going up, tauFall going down
 *   hysteresis: enter contact when p >= enterProbability, leave when p <= exitProbability
 * </pre>
 *
 * Outputs: probability = highest p; opponent = the one in contact with the highest p (UNKNOWN if
 * that is the unidentified one); direction = toward that opponent; for UNKNOWN toward the closest
 * id-less track if one adds vision evidence, else the push direction.
 *
 * <p>Ids. Each hypothesis belongs to an id, so the ids must stay with their robots: a track that
 * changes id starts a new hypothesis from p = 0, and its old one is dropped. RulesGuard therefore
 * feeds this class episode keys ({@link EpisodeKeys}), not raw tracker ids. Tracks without a
 * usable id never get a hypothesis of their own (they could not be told apart from loop to loop);
 * their camera evidence goes to UNKNOWN, which is never confused with them because UNKNOWN is
 * its own hypothesis, not an id in the list.
 *
 * <p>Deterministic: no clock reads (time is passed in), no randomness. It keeps state between
 * calls (smoothing, hysteresis, the fading deceleration cue), so use one instance per robot and
 * call it once per loop.
 */
public final class ContactDetector {
  /** Id used for "touching something we cannot identify". */
  public static final int UNKNOWN = -1;

  /**
   * Every threshold of the detector. See the class comment for where each one is used.
   *
   * @param pushMinSpeed below this commanded speed we are not "pushing" (m/s)
   * @param stallProgressNone progress at or above this is normal driving
   * @param stallProgressFull progress at or below this is a full stall
   * @param freeAccel acceleration along the command that means "speeding up freely" (m/s^2)
   * @param currentLow mean drive current at or below this is normal (A)
   * @param currentHigh mean drive current at or above this is a full push (A)
   * @param decelLow deceleration at or below this is normal braking (m/s^2)
   * @param decelHigh deceleration at or above this is a hit (m/s^2)
   * @param decelHoldSec how long a hit stays in the fusion (s)
   * @param visionGapNear estimated bumper gap at or below this is "touching" (m)
   * @param visionGapFar estimated bumper gap at or above this is "not touching" (m)
   * @param attributeGap an opponent within this estimated gap can be what we are pushing on (m)
   * @param attributeCos ... if it is also within this cosine of the push direction
   * @param bias log-odds of contact with no evidence
   * @param wStall log-odds added by a full stall cue
   * @param wCurrent log-odds added by a full current cue
   * @param wDecel log-odds added by a full deceleration cue
   * @param wVision log-odds added by a full vision cue
   * @param tauRise smoothing time constant while the probability rises (s)
   * @param tauFall smoothing time constant while it falls (s)
   * @param enterProbability enter contact at or above this
   * @param exitProbability leave contact at or below this
   */
  public record Config(
      double pushMinSpeed,
      double stallProgressNone,
      double stallProgressFull,
      double freeAccel,
      double currentLow,
      double currentHigh,
      double decelLow,
      double decelHigh,
      double decelHoldSec,
      double visionGapNear,
      double visionGapFar,
      double attributeGap,
      double attributeCos,
      double bias,
      double wStall,
      double wCurrent,
      double wDecel,
      double wVision,
      double tauRise,
      double tauFall,
      double enterProbability,
      double exitProbability) {

    /**
     * Rejects configs the formulas cannot use. Every ramp needs a width, the right way round
     * (ramp divides by oneAt - zeroAt: a zero width turns a far robot into "touching" and gives
     * 0 / 0 = NaN at the edge, and a NaN never leaves a hypothesis), and every divisor must be
     * positive:
     *
     * <pre>
     *   pushMinSpeed > 0              (stall cue divides by |v_cmd| >= pushMinSpeed)
     *   stallProgressNone > stallProgressFull,  freeAccel > 0
     *   currentHigh > currentLow,  decelHigh > decelLow,  decelHoldSec > 0
     *   visionGapFar > visionGapNear
     *   tauRise >= 0,  tauFall >= 0,  0 <= exitProbability <= enterProbability <= 1
     *   every number finite
     * </pre>
     */
    public Config {
      double[] all = {
        pushMinSpeed, stallProgressNone, stallProgressFull, freeAccel, currentLow, currentHigh,
        decelLow, decelHigh, decelHoldSec, visionGapNear, visionGapFar, attributeGap, attributeCos,
        bias, wStall, wCurrent, wDecel, wVision, tauRise, tauFall, enterProbability, exitProbability
      };
      for (double v : all) {
        if (!Double.isFinite(v)) {
          throw new IllegalArgumentException("contact detector settings must be finite numbers");
        }
      }
      require(pushMinSpeed > 0.0, "pushMinSpeed must be > 0");
      require(stallProgressNone > stallProgressFull, "stallProgressNone must be above Full");
      require(freeAccel > 0.0, "freeAccel must be > 0");
      require(currentHigh > currentLow, "currentHigh must be above currentLow");
      require(decelHigh > decelLow, "decelHigh must be above decelLow");
      require(decelHoldSec > 0.0, "decelHoldSec must be > 0");
      require(visionGapFar > visionGapNear,
          "visionGapFar must be above visionGapNear (a vision error must be > 0)");
      require(tauRise >= 0.0 && tauFall >= 0.0, "smoothing time constants must be >= 0");
      require(exitProbability >= 0.0 && exitProbability <= enterProbability
          && enterProbability <= 1.0, "need 0 <= exitProbability <= enterProbability <= 1");
    }

    private static void require(boolean ok, String message) {
      if (!ok) {
        throw new IllegalArgumentException(message);
      }
    }

    /**
     * PLACEHOLDERS, every one of them, tuned only against the simulator. Set them from real
     * measurements with the calibration procedure in docs/findings/contact-sensing.md before
     * trusting the detector in a match.
     */
    public static Config placeholder() {
      return new Config(
          0.4, // pushMinSpeed: just under the 0.45 m/s the sim referee calls "driving into"
          0.60, // stallProgressNone
          0.25, // stallProgressFull
          3.0, // freeAccel: half of the 6 m/s^2 the sim robot accelerates at
          20.0, // currentLow
          40.0, // currentHigh (current limit is 60 A in the sim)
          8.0, // decelLow: above the 6 m/s^2 the robot can brake at
          20.0, // decelHigh
          0.3, // decelHoldSec
          0.05, // visionGapNear: same tolerance the referee uses
          0.45, // visionGapFar: typical camera estimate error is 0.4 m
          0.6, // attributeGap
          0.0, // attributeCos: anywhere in the half-plane we are pushing toward
          -4.0, // bias: p = 0.018 with no evidence
          3.5, // wStall
          3.5, // wCurrent: stall + current -> L = 3, p = 0.95
          3.5, // wDecel
          5.5, // wVision: camera alone, gap <= 0.05 -> L = 1.5, p = 0.82
          0.02, // tauRise
          0.10, // tauFall
          0.7, // enterProbability
          0.3); // exitProbability
    }

    /**
     * Same, with the vision ramp set for a camera with this typical position error (m):
     * visionGapFar = visionGapNear + error. On the robot, use the measured tracking error.
     *
     * @throws IllegalArgumentException if errorM is not a positive number (a zero-width ramp
     *     would call a robot 1 m away "touching")
     */
    public Config withVisionError(double errorM) {
      return new Config(
          pushMinSpeed, stallProgressNone, stallProgressFull, freeAccel, currentLow, currentHigh,
          decelLow, decelHigh, decelHoldSec, visionGapNear, visionGapNear + errorM, attributeGap,
          attributeCos, bias, wStall, wCurrent, wDecel, wVision, tauRise, tauFall,
          enterProbability, exitProbability);
    }
  }

  private static final class Hypothesis {
    double p = 0.0;
    boolean inContact = false;
  }

  private final Config config;
  private final double robotHalfSize;
  private final Map<Integer, Hypothesis> byOpponent = new HashMap<>();
  private final Hypothesis unknown = new Hypothesis();
  private final Hypothesis blocked = new Hypothesis();
  private double decelHeld = 0.0;
  private Vec2 unknownDirection = Vec2.ZERO;
  private double lastTime = Double.NaN;

  /**
   * @param robotHalfSize half of the bumper-to-bumper width of a robot (m)
   */
  public ContactDetector(Config config, double robotHalfSize) {
    this.config = config;
    this.robotHalfSize = robotHalfSize;
  }

  public Config config() {
    return config;
  }

  /**
   * Linear ramp from 0 at {@code zeroAt} to 1 at {@code oneAt}, clamped to [0, 1]. Needs
   * zeroAt != oneAt ({@link Config} guarantees that for every ramp the detector uses).
   */
  public static double ramp(double x, double zeroAt, double oneAt) {
    double t = (x - zeroAt) / (oneAt - zeroAt);
    return Math.max(0.0, Math.min(1.0, t));
  }

  /** 1 / (1 + exp(-x)). */
  public static double sigmoid(double x) {
    return 1.0 / (1.0 + Math.exp(-x));
  }

  /** True if we are commanding at least pushMinSpeed. */
  public boolean pushing(DriveTelemetry t) {
    return t.commandedVelocity().norm() >= config.pushMinSpeed();
  }

  /** 1 if we are accelerating freely along the command (so not blocked), 0 if not at all. */
  public double speedingUp(DriveTelemetry t) {
    Vec2 u = t.commandedVelocity().normalized();
    return ramp(t.measuredAcceleration().dot(u), 0.0, config.freeAccel());
  }

  /** Cue (a): pushing but not moving. */
  public double stallCue(DriveTelemetry t) {
    if (!pushing(t)) {
      return 0.0;
    }
    Vec2 cmd = t.commandedVelocity();
    double progress = t.measuredVelocity().dot(cmd.normalized()) / cmd.norm();
    return ramp(progress, config.stallProgressNone(), config.stallProgressFull())
        * (1.0 - speedingUp(t));
  }

  /** Cue (b): drive current above threshold (falls back to the stall cue if not measured). */
  public double currentCue(DriveTelemetry t) {
    if (!pushing(t)) {
      return 0.0;
    }
    if (!t.hasCurrent()) {
      return stallCue(t);
    }
    if (braking(t)) {
      return 0.0; // the motors are slowing us down on purpose, not pushing on anything
    }
    return ramp(t.driveCurrentAmps(), config.currentLow(), config.currentHigh())
        * (1.0 - speedingUp(t));
  }

  /** Cue (c) for this one sample, before the hold: sudden deceleration we did not ask for. */
  public double decelCueNow(DriveTelemetry t) {
    if (!pushing(t)) {
      return 0.0;
    }
    if (braking(t)) {
      return 0.0; // faster than asked: slowing down is what the controller wants
    }
    Vec2 u = t.commandedVelocity().normalized();
    return ramp(-t.measuredAcceleration().dot(u), config.decelLow(), config.decelHigh());
  }

  /** True if we are moving faster along the command than commanded: v_meas . u > |v_cmd|. */
  public static boolean braking(DriveTelemetry t) {
    Vec2 cmd = t.commandedVelocity();
    return t.measuredVelocity().dot(cmd.normalized()) > cmd.norm();
  }

  /** Cue (d): how close the camera says this opponent is (1 = touching). */
  public double visionCue(double gap) {
    return ramp(gap, config.visionGapFar(), config.visionGapNear());
  }

  /** True if an opponent at {@code opponent} can be the thing we are pushing on. */
  public boolean explains(DriveTelemetry t, Vec2 self, Vec2 opponent) {
    if (!pushing(t)) {
      return false;
    }
    Vec2 rel = opponent.minus(self);
    double gap = rel.norm() - 2.0 * robotHalfSize;
    double cos = t.commandedVelocity().normalized().dot(rel.normalized());
    return gap <= config.attributeGap() && cos >= config.attributeCos();
  }

  /**
   * Call once per loop.
   *
   * @param nowSec current time (s)
   * @param telemetry this loop's drive telemetry
   * @param self our position (m)
   * @param opponents tracked opponents (may be empty: the drivetrain cues still work)
   */
  public ContactEstimate update(
      double nowSec, DriveTelemetry telemetry, Vec2 self, List<TrackedRobot> opponents) {
    double dt = Double.isNaN(lastTime) ? 0.0 : Math.max(0.0, nowSec - lastTime);
    lastTime = nowSec;

    double stall = stallCue(telemetry);
    double current = currentCue(telemetry);
    decelHeld = Math.max(decelCueNow(telemetry), decelHeld - dt / config.decelHoldSec());
    decelHeld = Math.max(0.0, decelHeld);
    double physical =
        config.wStall() * stall + config.wCurrent() * current + config.wDecel() * decelHeld;
    Vec2 push = pushing(telemetry) ? telemetry.commandedVelocity().normalized() : Vec2.ZERO;

    Map<Integer, Vec2> toward = new HashMap<>();
    Set<Integer> seen = new HashSet<>();
    List<TrackedRobot> identified = new java.util.ArrayList<>();
    boolean explained = false;
    double idlessVision = 0.0; // max over tracks without a usable id
    Vec2 idlessDirection = Vec2.ZERO;
    for (TrackedRobot r : opponents) {
      Vec2 pos = r.positionAt(nowSec);
      Vec2 rel = pos.minus(self);
      double gap = rel.norm() - 2.0 * robotHalfSize;
      if (r.id() < 0 || seen.contains(r.id())) {
        double v = visionCue(gap);
        if (v > idlessVision) {
          idlessVision = v;
          idlessDirection = rel.normalized();
        }
        continue;
      }
      boolean explains = explains(telemetry, self, pos);
      explained |= explains;
      double logit = config.bias() + config.wVision() * visionCue(gap) + (explains ? physical : 0.0);
      step(byOpponent.computeIfAbsent(r.id(), k -> new Hypothesis()), sigmoid(logit), dt);
      toward.put(r.id(), rel.normalized());
      seen.add(r.id());
      identified.add(r);
    }
    byOpponent.keySet().retainAll(seen); // a lost track cannot be confirmed: its contact ends

    double unknownRaw =
        sigmoid(config.bias() + config.wVision() * idlessVision + (explained ? 0.0 : physical));
    step(unknown, unknownRaw, dt);
    if (idlessVision > 0.0) {
      unknownDirection = idlessDirection;
    } else if (unknownRaw >= config.exitProbability() && !push.equals(Vec2.ZERO)) {
      unknownDirection = push;
    }
    step(blocked, sigmoid(config.bias() + physical), dt);

    Set<Integer> contacts = new HashSet<>();
    Map<Integer, Double> probabilities = new HashMap<>();
    probabilities.put(UNKNOWN, unknown.p);
    double best = unknown.p;
    int bestInContact = UNKNOWN;
    double bestInContactP = unknown.inContact ? unknown.p : -1.0;
    if (unknown.inContact) {
      contacts.add(UNKNOWN);
    }
    for (TrackedRobot r : identified) { // list order, so ties resolve the same way every run
      Hypothesis h = byOpponent.get(r.id());
      probabilities.put(r.id(), h.p);
      best = Math.max(best, h.p);
      if (h.inContact) {
        contacts.add(r.id());
        if (h.p > bestInContactP) {
          bestInContactP = h.p;
          bestInContact = r.id();
        }
      }
    }
    boolean inContact = !contacts.isEmpty();
    Vec2 direction = Vec2.ZERO;
    if (inContact) {
      direction = bestInContact == UNKNOWN ? unknownDirection : toward.get(bestInContact);
    }
    return new ContactEstimate(
        inContact,
        best,
        direction,
        bestInContact,
        blocked.inContact,
        push,
        contacts,
        probabilities,
        stall,
        current,
        decelHeld);
  }

  /** Smoothing and hysteresis for one hypothesis (see the class comment). */
  private void step(Hypothesis h, double raw, double dt) {
    double tau = raw > h.p ? config.tauRise() : config.tauFall();
    if (dt > 0.0) {
      h.p += (raw - h.p) * dt / (tau + dt);
    }
    if (h.p >= config.enterProbability()) {
      h.inContact = true;
    } else if (h.p <= config.exitProbability()) {
      h.inContact = false;
    }
  }
}
