package phantom.core.control;

import java.util.ArrayList;
import java.util.EnumMap;
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
import phantom.core.rules.EscapePlanner;
import phantom.core.rules.PinRules;
import phantom.core.rules.PinTimer;
import phantom.core.tracking.TrackedRobot;

/**
 * Keeps the assist legal. Runs after the mode, every loop, and overrides the target when a pin is
 * about to exceed the limit.
 *
 * <pre>
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
 *
 *   BACK OFF from id when contactSeconds(id) >= limit - safetyMargin, until a fresh estimate
 *   says gap >= resetSeparation (the episode itself stays open until the extra RESET_MARGIN, so
 *   touching again re-triggers at once), the episode ends, or MAX_BACKOFF_SECONDS pass. The direction comes from {@link EscapePlanner}: left, right or
 *   straight back, whichever does not hit a wall within D = BACKOFF_SPEED * LOOKAHEAD_SECONDS,
 *   never drives toward the opponent, leaves its path and has the most clearance. The direction
 *   is held (a straight line) for as long as it stays wall-free for D ahead of where we are now
 *   and still points away from every threat; when it does not, we pick again. If the drivetrain says we are blocked while backing off (after
 *   ESCAPE_GRACE_SECONDS to get moving), that option is ruled out for BLOCKED_SECONDS.
 *
 *     p*   = p_self + STEP * d,   v_ff = BACKOFF_SPEED * d
 *     no direction is free:  p* = p_self, v_ff = 0   (stop pushing; standing still is not a pin)
 *     still none after WEDGED_SECONDS: slide D along {@link EscapePlanner#leastInto}
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

  private static final int UNKNOWN = ContactDetector.UNKNOWN;

  private final PinTimer pinTimer;
  private final ContactDetector detector;
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
  private double wedgedSince = Double.NaN; // no free direction since then
  private boolean sliding = false; // last-resort slide out past a threat
  private Vec2 slideStart = Vec2.ZERO;
  private final Map<EscapePlanner.Option, Double> blockedUntil =
      new EnumMap<>(EscapePlanner.Option.class);

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

  public static RulesGuard defaults(FieldConfig field) {
    return new RulesGuard(PinRules.placeholder(), field, 0.5);
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
    ContactEstimate est = detector.update(nowSec, telemetry, selfPos, opponents);
    lastContact = est;

    Map<Integer, Vec2> positions = new HashMap<>();
    Map<Integer, Double> ages = new HashMap<>();
    for (TrackedRobot r : opponents) {
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
      sliding = false;
      wedgedSince = Double.NaN;
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
      } else if (nowSec - pinTimer.lastContactSec(UNKNOWN) <= HANDOVER_SECONDS) {
        pinTimer.raiseTo(id, pinTimer.contactSeconds(UNKNOWN));
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
    List<Vec2> others = new ArrayList<>();
    for (Map.Entry<Integer, Vec2> e : positions.entrySet()) {
      if (!backoffStarts.containsKey(e.getKey())) {
        others.add(e.getValue());
      }
    }
    double distance = BACKOFF_SPEED * LOOKAHEAD_SECONDS;

    // Blocked while backing off: rule this direction out for a while.
    if (escapeOption != null
        && nowSec - escapeSince >= ESCAPE_GRACE_SECONDS
        && est.blocked()
        && est.pushDirection().dot(escapeDirection) >= 0.7) {
      blockedUntil.put(escapeOption, nowSec + BLOCKED_SECONDS);
      escapeOption = null;
    }
    Set<EscapePlanner.Option> blocked = new HashSet<>();
    for (Map.Entry<EscapePlanner.Option, Double> e : blockedUntil.entrySet()) {
      if (nowSec < e.getValue()) {
        blocked.add(e.getKey());
      }
    }

    // Keep the current direction only while it stays free of walls for the whole look-ahead;
    // otherwise (or at the start) pick again from where we are now.
    boolean slideDone = sliding && selfPos.distanceTo(slideStart) >= distance;
    if (escapeOption != null
        && (slideDone
            || EscapePlanner.hitsWall(field, selfPos, escapeDirection, distance)
            || (!sliding && EscapePlanner.drivesInto(selfPos, threats, escapeDirection)))) {
      escapeOption = null;
      sliding = false;
    }
    if (escapeOption == null) {
      List<EscapePlanner.Candidate> candidates =
          EscapePlanner.candidates(field, selfPos, threats, distance, others);
      Optional<EscapePlanner.Candidate> choice = EscapePlanner.choose(candidates, blocked);
      if (choice.isEmpty()) {
        if (Double.isNaN(wedgedSince)) {
          wedgedSince = nowSec;
        }
        if (nowSec - wedgedSince >= WEDGED_SECONDS) {
          choice = EscapePlanner.leastInto(candidates, blocked);
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
    if (escapeOption == null) {
      return new DefenseTarget(selfPos, heading, Vec2.ZERO, nearest); // nowhere to go: stop pushing
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

  /** Pin time the guard has counted against this opponent (or UNKNOWN) in the open episode (s). */
  public double contactSeconds(int id) {
    return pinTimer.contactSeconds(id);
  }
}
