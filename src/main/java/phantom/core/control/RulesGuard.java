package phantom.core.control;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseTarget;
import phantom.core.rules.ContactDetector;
import phantom.core.rules.ContactEstimate;
import phantom.core.rules.DriveTelemetry;
import phantom.core.rules.EpisodeKeys;
import phantom.core.rules.EscapePlanner;
import phantom.core.rules.PinRules;
import phantom.core.rules.PinTimer;
import phantom.core.tracking.TrackedRobot;

/**
 * Keeps the assist legal. Runs after the mode, every loop, and overrides the target when a pin is
 * about to exceed the limit.
 *
 * <pre>
 *   EPISODE KEYS. Everything below is keyed by contact episode, not by raw track id: every loop
 *   the tracks are re-labelled by {@link EpisodeKeys} (rebind radius REBIND_RADIUS, freshness
 *   FRESH_SECONDS). A tracker's own handoff keeps the id, so the key stays; a robot that comes
 *   back under a NEW id near where an open episode's opponent was lost continues that episode
 *   (its pin time and the detector's contact belief). Only for trackers whose ids can change
 *   ({@link #setIdsAreStable}; DefenseAssist sets it from OpponentTracker.idsAreStable()).
 *   With stable ids every key is the id itself. Ids in the outputs (targets, lastContact,
 *   contactSeconds) are the current track ids again.
 *
 *   CONTACT comes from {@link ContactDetector} (stall + drive current + deceleration + camera):
 *     contact with opponent i   = detector is in contact with i
 *     contact with UNKNOWN      = detector is in contact with something it cannot identify,
 *                                 unless a wall explains it: one WALL_PROBE step along the push
 *                                 direction, clamped to the field, makes <= WALL_PROGRESS of it
 *
 *   PIN TIME per opponent (and for UNKNOWN) accumulates while in contact. The episode ends when
 *     tracked opponent:  a camera estimate at most FRESH_SECONDS old says
 *                        gap >= resetSeparation + RESET_MARGIN
 *     not tracked:       no contact for FORGET_SECONDS
 *     UNKNOWN:           no contact for FORGET_SECONDS
 *   Handover, so time is not lost when the camera gains or loses the robot we are pushing:
 *     a new touch on opponent i starts from UNKNOWN's time if UNKNOWN was touching within
 *     HANDOVER_SECONDS; a new UNKNOWN touch starts from the largest time among opponents that
 *     are no longer tracked or stopped touching within HANDOVER_SECONDS.
 *     If ids can change (anonymous tracking), a new touch on i also starts from the time of any
 *     other episode whose track is lost or stale (older than FRESH_SECONDS) and that was
 *     touching within HANDOVER_SECONDS: the robot we were pushing, back under another track.
 *
 *   BACK OFF from id when contactSeconds(id) >= limit - safetyMargin (safetyMargin from
 *   {@link #safetyMarginFor}: slack + latency + one loop + time to slide one robot width out at
 *   this robot's speed and acceleration), until a fresh estimate says gap >= resetSeparation
 *   (the episode itself stays open until the extra RESET_MARGIN, so touching again re-triggers
 *   at once), the episode ends, or MAX_BACKOFF_SECONDS pass.
 *
 *   Which way: {@link EscapePlanner} with every robot we see, predicted over the look-ahead
 *     D = max(v * LOOKAHEAD_SECONDS, 2 * robotHalfSize + CONTACT_TOLERANCE),
 *     v = min(speed cap, BACKOFF_SPEED), acceleration from {@link #setDriveLimits}.
 *   A robot counts as touching if the detector says so or its camera gap is <= TOUCH_GAP.
 *     1. the best usable LEFT / RIGHT / BACK ({@link EscapePlanner#chooseExit}): no wall, no
 *        push on anyone we touch, not along a pusher's path, not into another robot;
 *     2. else the best usable extra direction (perpendiculars to each pusher's path, 16 compass);
 *     3. else YIELD: p* = p_self, v_ff = 0 (zero push effort toward every robot; whoever pushes
 *        moves us). If no exit appears for WEDGED_SECONDS, slide D along
 *        {@link EscapePlanner#leastInto} over all of them (the shallowest angle past the threats).
 *   The direction is held while it stays usable from where we are now (a slide: until it has
 *   gone D or meets a wall). If the drivetrain says we are blocked while backing off (after
 *   ESCAPE_GRACE_SECONDS), directions within 30 deg of it are ruled out for BLOCKED_SECONDS.
 *
 *     p* = p_self + STEP * d,   v_ff = BACKOFF_SPEED * d
 *
 *   LOOK. If the mode has no target (nobody to guard) and we touched something the camera
 *   does not see within LOOK_SECONDS, stand still and face it instead of driving on blind:
 *     p* = p_self, heading = toward the contact, v_ff = 0
 * </pre>
 *
 * Stateful on purpose (it measures time), so it lives here and not in a DefenseMode.
 */
public final class RulesGuard {
  public static final double MAX_BACKOFF_SECONDS = 3.0;
  public static final double STEP = 1.0;
  public static final double BACKOFF_SPEED = 3.0;

  /**
   * Real slack the guard keeps below the pin limit: it aims for at most limit - PEAK_SLACK of
   * contact (2.5 s with the 3 s placeholder limit). See {@link #safetyMarginFor}.
   */
  public static final double PEAK_SLACK = 0.5;

  /** One control loop: a back-off decided now is first commanded on the next loop (s). */
  public static final double LOOP_SECONDS = 0.02;

  /** Speed and acceleration of the simulated robot (m/s, m/s^2). */
  public static final double SIM_SPEED = 4.5;

  public static final double SIM_ACCEL = 6.0;

  /**
   * A robot at most this far (bumper gap, m) by the camera counts as touching for the back-off's
   * crowd checks, as well as anything the contact detector says we touch.
   */
  public static final double TOUCH_GAP = 0.1;

  /** Command latency of the simulated robot (s): none beyond the loop. */
  public static final double SIM_LATENCY_SECONDS = 0.0;

  /** How far ahead a back-off direction must be free of walls (s of travel at BACKOFF_SPEED). */
  public static final double LOOKAHEAD_SECONDS = 0.5;

  /** PLACEHOLDER. Extra separation, beyond the rule, before a camera estimate ends an episode (m). */
  public static final double RESET_MARGIN = 0.3;

  /** PLACEHOLDER. A camera estimate older than this cannot end an episode (s). */
  public static final double FRESH_SECONDS = 0.25;

  /** PLACEHOLDER. An opponent we cannot see is assumed gone after this long without contact (s). */
  public static final double FORGET_SECONDS = 2.0;

  /** PLACEHOLDER. Window for passing pin time between UNKNOWN and an identified opponent (s). */
  public static final double HANDOVER_SECONDS = 0.5;

  /** PLACEHOLDER. With nothing to guard, stand still and look at an unseen contact this long (s). */
  public static final double LOOK_SECONDS = 1.0;

  /**
   * PLACEHOLDER. Wedged (no direction free) this long: slide out past the threat at the shallowest
   * angle, for one look-ahead distance (s).
   */
  public static final double WEDGED_SECONDS = 1.0;

  /** Time allowed to get moving in a new back-off direction before "blocked" counts (s). */
  public static final double ESCAPE_GRACE_SECONDS = 0.2;

  /** How long a blocked back-off direction stays ruled out (s). */
  public static final double BLOCKED_SECONDS = 1.0;

  /** Step used to ask "would a wall alone stop this push?" (m). */
  public static final double WALL_PROBE = 0.1;

  /** A wall explains a stall if the probe step makes at most this fraction of its length. */
  public static final double WALL_PROGRESS = 0.3;

  /**
   * PLACEHOLDER. A track with a new id this close to where a lost episode's opponent was last
   * seen fresh continues that episode (m). About two Realistic-camera position errors (0.4-0.5 m).
   */
  public static final double REBIND_RADIUS = 1.0;

  private static final int UNKNOWN = ContactDetector.UNKNOWN;

  private final PinTimer pinTimer;
  private final ContactDetector detector;
  private final EpisodeKeys keys = new EpisodeKeys(REBIND_RADIUS, FRESH_SECONDS, true);
  private boolean idsAreStable = false;
  private final FieldConfig field;
  private final double safetyMargin;
  private final Map<Integer, Double> backoffStarts = new HashMap<>();
  private final Set<Integer> released = new HashSet<>(); // back-off ended apart, episode still open
  private final Map<Integer, EscapePlanner.Threat> lastSeen = new HashMap<>();
  private int backoffsTriggered = 0;
  private ContactEstimate lastContact = ContactEstimate.none();
  private boolean timingContact = false;

  // UNKNOWN blocker: where we were and which way we pushed when we last felt it.
  private Vec2 unknownAnchor = null;
  private Vec2 unknownDirection = Vec2.ZERO;
  private boolean unknownIsWall = false;

  // Current back-off leg.
  private EscapePlanner.Option escapeOption = null;
  private Vec2 escapeDirection = Vec2.ZERO;
  private double escapeSince = Double.NaN;
  private boolean yielding = false; // no safe exit: not pushing on anyone
  private double wedgedSince = Double.NaN; // no safe exit since then
  private boolean sliding = false; // last-resort slide out past the threats
  private Vec2 slideStart = Vec2.ZERO;
  private final List<Vec2> blockedDirections = new ArrayList<>();
  private final List<Double> blockedUntil = new ArrayList<>();

  // How this robot drives while backing off (setDriveLimits).
  private EscapePlanner.Motion motion = new EscapePlanner.Motion(BACKOFF_SPEED, SIM_ACCEL);

  // For callers that pass velocities instead of telemetry.
  private Vec2 previousMeasured = Vec2.ZERO;
  private double previousTime = Double.NaN;

  /**
   * @param rules pin limits from the game manual
   * @param field field geometry (robot size, walls)
   * @param safetyMargin start backing off this long before the limit (s)
   */
  public RulesGuard(PinRules rules, FieldConfig field, double safetyMargin) {
    this(rules, field, safetyMargin, ContactDetector.Config.placeholder());
  }

  public RulesGuard(
      PinRules rules, FieldConfig field, double safetyMargin, ContactDetector.Config detectorConfig) {
    this.pinTimer = new PinTimer(rules, field.robotHalfSize());
    this.detector = new ContactDetector(detectorConfig, field.robotHalfSize());
    this.field = field;
    this.safetyMargin = safetyMargin;
  }

  /**
   * Time to drive {@code distance} from standing still, accelerating at {@code accel} up to
   * {@code speed} (trapezoid, no braking at the end):
   *
   * <pre>
   *   d_acc = speed^2 / (2 accel)                       distance covered while speeding up
   *   T     = sqrt(2 distance / accel)                  if distance <= d_acc
   *         = speed / accel + (distance - d_acc) / speed  otherwise
   * </pre>
   *
   * +infinity if speed or accel is not a positive finite number (or distance is negative).
   */
  public static double moveSeconds(double distance, double speed, double accel) {
    if (!(speed > 0.0) || !(accel > 0.0) || !Double.isFinite(speed) || !Double.isFinite(accel)
        || !(distance >= 0.0)) {
      return Double.POSITIVE_INFINITY;
    }
    double dAcc = speed * speed / (2.0 * accel);
    if (distance <= dAcc) {
      return Math.sqrt(2.0 * distance / accel);
    }
    return speed / accel + (distance - dAcc) / speed;
  }

  /**
   * How long before the pin limit to start backing off, for a robot that backs off at most at
   * {@code maxSpeed} with acceleration {@code maxAccel} and whose commands take
   * {@code latencySec} to reach the wheels. The same rule runs in the simulator and on the robot.
   *
   * <pre>
   *   v      = min(maxSpeed, BACKOFF_SPEED)          (the back-off never asks for more)
   *   d      = 2 * robotHalfSize + CONTACT_TOLERANCE  (slide one robot width out of a head-on
   *                                                   push, plus the contact tolerance)
   *   margin = PEAK_SLACK + latencySec + LOOP_SECONDS + moveSeconds(d, v, maxAccel)
   *   clamped to at most limitSeconds (then the guard backs off as soon as contact starts)
   * </pre>
   *
   * So contact should end by limit - PEAK_SLACK even if the back-off has to cover a whole robot
   * width from standing still. Examples (half size 0.45 m, d = 0.95 m, limit 3 s):
   * sim robot (3 m/s back-off, 6 m/s^2, no latency): 0.5 + 0 + 0.02 + 0.5667 = 1.0867 s;
   * Phase 7 first test (1 m/s, 2 m/s^2, 0.1 s): 0.5 + 0.1 + 0.02 + 1.2 = 1.82 s.
   */
  public static double safetyMarginFor(
      PinRules rules, double robotHalfSize, double maxSpeed, double maxAccel, double latencySec) {
    double v = Math.min(maxSpeed, BACKOFF_SPEED);
    double d = 2.0 * robotHalfSize + PinTimer.CONTACT_TOLERANCE;
    double margin =
        PEAK_SLACK + Math.max(0.0, latencySec) + LOOP_SECONDS + moveSeconds(d, v, maxAccel);
    return Double.isFinite(margin) ? Math.min(margin, rules.limitSeconds()) : rules.limitSeconds();
  }

  /** The margin for the simulator's robot (4.5 m/s, 6 m/s^2, no extra latency): 1.0867 s. */
  public static double simMargin(PinRules rules, double robotHalfSize) {
    return safetyMarginFor(rules, robotHalfSize, SIM_SPEED, SIM_ACCEL, SIM_LATENCY_SECONDS);
  }

  /**
   * How this robot drives while backing off: at v = min(maxSpeed, BACKOFF_SPEED) with
   * acceleration maxAccel. Defaults to the sim robot (3 m/s, 6 m/s^2). Non-positive or NaN values
   * are ignored.
   */
  public void setDriveLimits(double maxSpeed, double maxAccel) {
    if (maxSpeed > 0.0 && maxAccel > 0.0) {
      motion = new EscapePlanner.Motion(Math.min(maxSpeed, BACKOFF_SPEED), maxAccel);
    }
  }

  /**
   * Look-ahead distance of the back-off (m): LOOKAHEAD_SECONDS of travel at this robot's back-off
   * speed, but never less than one robot width plus the contact tolerance (the slide out of a
   * head-on push):
   *
   * <pre>
   *   D = max(v * LOOKAHEAD_SECONDS, 2 * robotHalfSize + CONTACT_TOLERANCE)
   * </pre>
   *
   * Sim robot: max(3 * 0.5, 0.95) = 1.5 m. Phase 7 first test (1 m/s): max(0.5, 0.95) = 0.95 m.
   */
  public double lookahead() {
    return Math.max(
        motion.speed() * LOOKAHEAD_SECONDS,
        2.0 * field.robotHalfSize() + PinTimer.CONTACT_TOLERANCE);
  }

  /** Back off this long before the limit (s). */
  public double safetyMargin() {
    return safetyMargin;
  }

  public static RulesGuard defaults(FieldConfig field) {
    return new RulesGuard(
        PinRules.placeholder(), field, simMargin(PinRules.placeholder(), field.robotHalfSize()));
  }

  /** Camera only: no drive telemetry this loop (we are treated as not pushing). */
  public Optional<DefenseTarget> apply(
      double nowSec, Pose self, List<TrackedRobot> opponents, Optional<DefenseTarget> target) {
    return apply(nowSec, self, opponents, target, DriveTelemetry.idle(nowSec));
  }

  /**
   * For callers with velocities but no IMU or current reading. Acceleration is taken from
   * successive calls; the detector's current cue falls back to its stall cue.
   *
   * @param commanded our last commanded field velocity (m/s)
   * @param actual our measured field velocity (m/s)
   */
  public Optional<DefenseTarget> apply(
      double nowSec,
      Pose self,
      List<TrackedRobot> opponents,
      Optional<DefenseTarget> target,
      Vec2 commanded,
      Vec2 actual) {
    double dt = Double.isNaN(previousTime) ? 0.0 : nowSec - previousTime;
    DriveTelemetry telemetry =
        DriveTelemetry.fromVelocities(nowSec, commanded, actual, previousMeasured, dt);
    previousMeasured = actual;
    previousTime = nowSec;
    return apply(nowSec, self, opponents, target, telemetry);
  }

  /** Update the detector and timers and return the (possibly overridden) target. Call every loop. */
  public Optional<DefenseTarget> apply(
      double nowSec,
      Pose self,
      List<TrackedRobot> opponents,
      Optional<DefenseTarget> target,
      DriveTelemetry telemetry) {
    Vec2 selfPos = self.position();
    // From here on every "id" is an episode key (see the class comment).
    List<TrackedRobot> keyed = keys.rekey(nowSec, opponents, pinTimer::episodeOpen);
    ContactEstimate est = detector.update(nowSec, telemetry, selfPos, keyed);
    lastContact = toTrackIds(est);

    Map<Integer, Vec2> positions = new HashMap<>();
    Map<Integer, Double> ages = new HashMap<>();
    for (TrackedRobot r : keyed) {
      positions.put(r.id(), r.positionAt(nowSec));
      ages.put(r.id(), nowSec - r.timestampSec());
      lastSeen.put(r.id(), new EscapePlanner.Threat(r.positionAt(nowSec), r.velocity(), true));
    }

    // Is the unidentified thing we are pushing on just a wall?
    if (!est.pushDirection().equals(Vec2.ZERO)) {
      unknownIsWall = wallBlocks(field, selfPos, est.pushDirection());
    }
    boolean unknownContact = est.inContactWith(UNKNOWN) && !unknownIsWall;

    updateTimers(nowSec, selfPos, est, positions, ages, unknownContact);

    // Finish back-offs whose episode ended, that are far enough apart, or that timed out.
    double rule = pinTimer.rules().resetSeparation();
    backoffStarts
        .entrySet()
        .removeIf(
            e -> {
              int id = e.getKey();
              boolean apart =
                  positions.containsKey(id)
                      && ages.get(id) <= FRESH_SECONDS
                      && pinTimer.gap(selfPos, positions.get(id)) >= rule;
              if (apart) {
                released.add(id);
              }
              return !pinTimer.episodeOpen(id)
                  || apart
                  || nowSec - e.getValue() >= MAX_BACKOFF_SECONDS;
            });
    released.removeIf(id -> !pinTimer.episodeOpen(id));

    // Start back-offs for everything close to the limit.
    double trigger = pinTimer.rules().limitSeconds() - safetyMargin;
    for (int id : pinTimer.openEpisodes()) {
      boolean armed = !released.contains(id) || pinTimer.touching(id); // released: new touch only
      if (!backoffStarts.containsKey(id) && armed && pinTimer.contactSeconds(id) >= trigger) {
        released.remove(id);
        backoffStarts.put(id, nowSec);
        backoffsTriggered++;
      }
    }
    if (backoffStarts.isEmpty()) {
      escapeOption = null;
      yielding = false;
      sliding = false;
      wedgedSince = Double.NaN;
      blockedDirections.clear();
      blockedUntil.clear();
      if (target.isEmpty()) {
        Optional<Vec2> unseen = unseenContact(nowSec, selfPos, positions);
        if (unseen.isPresent()) { // stop and look at what we hit
          return Optional.of(new DefenseTarget(selfPos, unseen.get().angle(), Vec2.ZERO, -1));
        }
      }
      return target;
    }
    return Optional.of(backOff(nowSec, self, est, positions));
  }

  private void updateTimers(
      double nowSec,
      Vec2 selfPos,
      ContactEstimate est,
      Map<Integer, Vec2> positions,
      Map<Integer, Double> ages,
      boolean unknownContact) {
    Set<Integer> ids = new TreeSet<>(positions.keySet());
    ids.addAll(pinTimer.openEpisodes());
    ids.add(UNKNOWN);

    // Handover at the start of a touch (see class comment).
    for (int id : ids) {
      boolean contact = id == UNKNOWN ? unknownContact : est.inContactWith(id);
      if (!contact || pinTimer.touching(id)) {
        continue;
      }
      if (id == UNKNOWN) {
        for (int other : pinTimer.openEpisodes()) {
          boolean recent = nowSec - pinTimer.lastContactSec(other) <= HANDOVER_SECONDS;
          if (other != UNKNOWN && (!positions.containsKey(other) || recent)) {
            pinTimer.raiseTo(UNKNOWN, pinTimer.contactSeconds(other));
          }
        }
      } else {
        if (nowSec - pinTimer.lastContactSec(UNKNOWN) <= HANDOVER_SECONDS) {
          pinTimer.raiseTo(id, pinTimer.contactSeconds(UNKNOWN));
        }
        if (!idsAreStable) { // the robot we were pushing may be back under a new id
          for (int other : pinTimer.openEpisodes()) {
            boolean lostOrStale = !positions.containsKey(other) || ages.get(other) > FRESH_SECONDS;
            if (other != UNKNOWN && other != id && lostOrStale
                && nowSec - pinTimer.lastContactSec(other) <= HANDOVER_SECONDS) {
              pinTimer.raiseTo(id, pinTimer.contactSeconds(other));
            }
          }
        }
      }
    }

    double reset = pinTimer.rules().resetSeparation();
    boolean blockedByRobot = est.blocked() && !unknownIsWall;
    pinTimer.updateContacts(
        nowSec,
        ids,
        id -> {
          if (id == UNKNOWN) {
            return unknownContact;
          }
          // An opponent we lost sight of mid-episode may be what blocks us now.
          return positions.containsKey(id) ? est.inContactWith(id) : blockedByRobot;
        },
        id -> {
          boolean forgotten = nowSec - pinTimer.lastContactSec(id) >= FORGET_SECONDS;
          if (id == UNKNOWN) {
            return forgotten; // it may be moving with us, so distance travelled proves nothing
          }
          if (!positions.containsKey(id)) {
            return forgotten;
          }
          return ages.get(id) <= FRESH_SECONDS
              && pinTimer.gap(selfPos, positions.get(id)) >= reset + RESET_MARGIN;
        });
    timingContact = false;
    for (int id : ids) {
      timingContact |= pinTimer.touching(id);
    }
    if (unknownContact) {
      unknownAnchor = selfPos;
      if (!est.pushDirection().equals(Vec2.ZERO)) {
        unknownDirection = est.pushDirection();
      }
    }
  }

  private DefenseTarget backOff(
      double nowSec, Pose self, ContactEstimate est, Map<Integer, Vec2> positions) {
    Vec2 selfPos = self.position();
    double reach = 2.0 * field.robotHalfSize();
    List<EscapePlanner.Threat> threats = new ArrayList<>();
    int nearest = -1;
    Vec2 nearestPos = null;
    for (int id : new TreeSet<>(backoffStarts.keySet())) {
      EscapePlanner.Threat threat;
      if (id == UNKNOWN) {
        Vec2 anchor = unknownAnchor == null ? selfPos : unknownAnchor;
        threat = new EscapePlanner.Threat(anchor.plus(unknownDirection.times(reach)), Vec2.ZERO, false);
      } else if (positions.containsKey(id)) {
        threat = lastSeen.get(id);
      } else {
        threat = new EscapePlanner.Threat(lastSeen.get(id).position(), Vec2.ZERO, false); // lost: stale
      }
      threats.add(threat);
      if (nearestPos == null || threat.position().distanceTo(selfPos) < nearestPos.distanceTo(selfPos)) {
        nearestPos = threat.position();
        nearest = positions.containsKey(id) ? id : -1;
      }
    }
    // Every other robot we see: touching if the detector says so or the camera gap is tiny.
    List<EscapePlanner.Robot> robots = new ArrayList<>();
    for (Map.Entry<Integer, Vec2> e : positions.entrySet()) {
      if (!backoffStarts.containsKey(e.getKey())) {
        boolean touching =
            est.inContactWith(e.getKey()) || pinTimer.gap(selfPos, e.getValue()) <= TOUCH_GAP;
        robots.add(new EscapePlanner.Robot(
            e.getValue(), lastSeen.get(e.getKey()).velocity(), true, touching));
      }
    }
    double distance = lookahead();

    // Blocked while backing off: rule this direction out for a while.
    if (escapeOption != null
        && nowSec - escapeSince >= ESCAPE_GRACE_SECONDS
        && est.blocked()
        && est.pushDirection().dot(escapeDirection) >= 0.7) {
      blockedDirections.add(escapeDirection);
      blockedUntil.add(nowSec + BLOCKED_SECONDS);
      escapeOption = null;
      sliding = false;
    }
    for (int i = blockedUntil.size() - 1; i >= 0; i--) {
      if (nowSec >= blockedUntil.get(i)) {
        blockedUntil.remove(i);
        blockedDirections.remove(i);
      }
    }

    // Keep the current direction while it is still a way out from where we are now (a slide:
    // until it has gone D or meets a wall); otherwise, or at the start, pick again.
    if (escapeOption != null && sliding) {
      if (selfPos.distanceTo(slideStart) >= distance
          || EscapePlanner.hitsWall(field, selfPos, escapeDirection, distance)) {
        escapeOption = null;
        sliding = false;
      }
    } else if (escapeOption != null
        && !EscapePlanner.evaluate(field, selfPos, threats, robots, escapeDirection, distance, motion)
            .usable()) {
      escapeOption = null;
    }
    if (escapeOption == null) {
      sliding = false;
      List<EscapePlanner.Candidate> classic =
          EscapePlanner.candidates(field, selfPos, threats, robots, distance, motion);
      List<EscapePlanner.Candidate> extra =
          EscapePlanner.extraCandidates(field, selfPos, threats, robots, distance, motion);
      Optional<EscapePlanner.Candidate> choice = EscapePlanner.chooseExit(classic, blockedDirections);
      if (choice.isEmpty()) {
        choice = EscapePlanner.chooseExit(extra, blockedDirections);
      }
      if (choice.isEmpty()) { // no safe exit: yield, and slide out if that goes on too long
        if (Double.isNaN(wedgedSince)) {
          wedgedSince = nowSec;
        }
        if (nowSec - wedgedSince >= WEDGED_SECONDS) {
          List<EscapePlanner.Candidate> all = new ArrayList<>(classic);
          all.addAll(extra);
          choice = EscapePlanner.leastInto(all, Set.of());
          sliding = choice.isPresent();
          slideStart = selfPos;
        }
      }
      if (choice.isPresent()) {
        escapeOption = choice.get().option();
        escapeDirection = choice.get().direction();
        escapeSince = nowSec;
        wedgedSince = Double.NaN;
      }
    }

    double heading = nearestPos.minus(selfPos).angle(); // keep facing them
    nearest = nearest == UNKNOWN ? UNKNOWN : keys.aliasOf(nearest); // back to a track id
    yielding = escapeOption == null;
    if (yielding) {
      // No safe exit: YIELD. Stand where we are with no feedforward, so the command pushes on
      // nobody (zero effort toward every robot); whoever pushes moves us.
      return new DefenseTarget(selfPos, heading, Vec2.ZERO, nearest);
    }
    Vec2 retreat = field.clampInside(selfPos.plus(escapeDirection.times(STEP)));
    return new DefenseTarget(retreat, heading, escapeDirection.times(BACKOFF_SPEED), nearest);
  }

  /**
   * Direction (from us) of something we touched within LOOK_SECONDS that the camera does not see:
   * the UNKNOWN blocker, or an opponent we lost sight of mid-episode. Empty if there is none.
   */
  private Optional<Vec2> unseenContact(double nowSec, Vec2 selfPos, Map<Integer, Vec2> positions) {
    Vec2 best = null;
    double bestTime = Double.NEGATIVE_INFINITY;
    for (int id : pinTimer.openEpisodes()) {
      double last = pinTimer.lastContactSec(id);
      if (positions.containsKey(id) || nowSec - last > LOOK_SECONDS || last <= bestTime) {
        continue;
      }
      Vec2 dir =
          id == UNKNOWN ? unknownDirection : lastSeen.get(id).position().minus(selfPos).normalized();
      if (!dir.equals(Vec2.ZERO)) {
        best = dir;
        bestTime = last;
      }
    }
    return Optional.ofNullable(best);
  }

  /**
   * True if a wall alone would stop a push from {@code self} along the unit vector {@code push}:
   *
   * <pre>
   *   q        = clampInside(self + WALL_PROBE * push)
   *   progress = (q - self) . push / WALL_PROBE
   *   wall blocks  <=>  progress <= WALL_PROGRESS
   * </pre>
   */
  public static boolean wallBlocks(FieldConfig field, Vec2 self, Vec2 push) {
    Vec2 q = field.clampInside(self.plus(push.times(WALL_PROBE)));
    return q.minus(self).dot(push) / WALL_PROBE <= WALL_PROGRESS;
  }

  /**
   * Old stall test, kept because existing tests use it. The guard itself now uses {@link
   * ContactDetector#stallCue}.
   *
   * <pre>
   *   blocked = |v_cmd| > 1.0  and  v_actual . unit(v_cmd) < 0.3 * |v_cmd|
   * </pre>
   */
  public static boolean isBlocked(Vec2 commanded, Vec2 actual) {
    double speed = commanded.norm();
    if (speed <= 1.0) {
      return false;
    }
    return actual.dot(commanded.normalized()) < 0.3 * speed;
  }

  /**
   * Old single-opponent back-off direction, kept because existing tests use it. The guard itself
   * now uses {@link EscapePlanner}.
   *
   * <pre>
   *   if |v_opp| > 0.3:  d = perp(unit(v_opp)), sign chosen so d . (p_self - p_opp) >= 0
   *   else:              d = unit(p_self - p_opp)
   * </pre>
   */
  public static Vec2 backoffDirection(Vec2 self, Vec2 opp, Vec2 oppVelocity) {
    Vec2 away = self.minus(opp).normalized();
    if (away.equals(Vec2.ZERO)) {
      away = new Vec2(1.0, 0.0);
    }
    if (oppVelocity.norm() <= 0.3) {
      return away;
    }
    Vec2 v = oppVelocity.normalized();
    Vec2 perp = new Vec2(-v.y(), v.x());
    return perp.dot(away) >= 0 ? perp : perp.times(-1.0);
  }

  public boolean backingOff() {
    return !backoffStarts.isEmpty();
  }

  /** True while backing off from something the camera has not identified. */
  public boolean blindBackingOff(double nowSec) {
    return backoffStarts.containsKey(UNKNOWN);
  }

  /** True while backing off with no safe exit: standing still, pushing on nobody. */
  public boolean yielding() {
    return backingOff() && yielding;
  }

  /** Which way the current back-off goes; empty if not backing off or standing still. */
  public Optional<EscapePlanner.Option> escapeOption() {
    return backingOff() ? Optional.ofNullable(escapeOption) : Optional.empty();
  }

  public int backoffsTriggered() {
    return backoffsTriggered;
  }

  /** True if, on the last {@code apply}, pin time was running against anyone (or UNKNOWN). */
  public boolean timingContact() {
    return timingContact;
  }

  /** The contact detector's output from the last {@code apply}. */
  public ContactEstimate lastContact() {
    return lastContact;
  }

  /**
   * Pin time the guard has counted in the open episode of this track id (or UNKNOWN) (s). A track
   * that continues an episode under a new id reports the episode's whole time.
   */
  public double contactSeconds(int id) {
    int key = keys.keyOf(id);
    return key == -2 ? 0.0 : pinTimer.contactSeconds(key);
  }

  /**
   * False (the default) for trackers that may give a robot a new id: a new id near a lost
   * opponent then continues its episode. True turns that off (a new id is a different robot).
   */
  public void setIdsAreStable(boolean stable) {
    idsAreStable = stable;
    keys.setRebindNewIds(!stable);
  }

  /** Times a track with a new id continued a lost opponent's episode. */
  public int episodeRebinds() {
    return keys.rebinds();
  }

  /** The detector's estimate with episode keys turned back into current track ids. */
  private ContactEstimate toTrackIds(ContactEstimate e) {
    Set<Integer> contacts = new HashSet<>();
    for (int k : e.contacts()) {
      contacts.add(k == UNKNOWN ? UNKNOWN : keys.aliasOf(k));
    }
    Map<Integer, Double> probabilities = new HashMap<>();
    for (Map.Entry<Integer, Double> en : e.probabilities().entrySet()) {
      int id = en.getKey() == UNKNOWN ? UNKNOWN : keys.aliasOf(en.getKey());
      probabilities.merge(id, en.getValue(), Math::max);
    }
    int opponent = e.opponentId() == UNKNOWN ? UNKNOWN : keys.aliasOf(e.opponentId());
    return new ContactEstimate(
        e.inContact(), e.probability(), e.direction(), opponent, e.blocked(), e.pushDirection(),
        contacts, probabilities, e.stallCue(), e.currentCue(), e.decelCue());
  }
}
