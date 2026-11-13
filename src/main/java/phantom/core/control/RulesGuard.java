package phantom.core.control;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseTarget;
import phantom.core.rules.PinRules;
import phantom.core.rules.PinTimer;
import phantom.core.tracking.TrackedRobot;

/**
 * Keeps the assist legal. Runs after the mode, every loop, and overrides the target when a pin is
 * about to exceed the limit.
 *
 * <pre>
 *   start backing off from opp i when contactSeconds(i) >= limit - safetyMargin
 *   (contact time is counted conservatively: any touching counts, even if they push us)
 *   contact = estimated gap <= 0.05 m, OR we are blocked (see below) and this is the nearest
 *   opponent within 2 * halfSize + BLOCKED_SEARCH_GAP. Vision estimates can be off by 0.4 m, so
 *   the robot's own "I'm pushing and not moving" signal catches contact that cameras miss.
 *
 *   per opponent i being backed off from, step ASIDE so a pushing opponent can pass:
 *     if |v_i| > 0.3:  d_i = perp(unit(v_i)), sign chosen so d_i . (p_self - p_i) >= 0
 *     else:            d_i = unit(p_self - p_i)                        (straight away)
 *   D    = unit(sum of d_i)          (several at once, e.g. when sandwiched)
 *   p*   = p_self + STEP * D,  v_ff = BACKOFF_SPEED * D
 *
 *   stop backing off from i when its pin episode ends (gap >= resetSeparation)
 *   or MAX_BACKOFF_SECONDS pass (e.g. a wall stops us)
 *
 *   blind fallback: if we have been blocked continuously for (limit - safetyMargin) against
 *   something we can't identify, reverse for BLIND_BACKOFF_SECONDS:
 *     p* = p_self - STEP * unit(v_cmd_when_blocked),  v_ff = -BACKOFF_SPEED * unit(...)
 * </pre>
 *
 * Stateful on purpose (it measures time), so it lives here and not in a DefenseMode.
 */
public final class RulesGuard {
  public static final double MAX_BACKOFF_SECONDS = 3.0;
  public static final double STEP = 1.0;
  public static final double BACKOFF_SPEED = 3.0;

  private final PinTimer pinTimer;
  private final FieldConfig field;
  private final double safetyMargin;
  private final Map<Integer, Double> backoffStarts = new HashMap<>();
  private int backoffsTriggered = 0;
  private double blockedSince = Double.NaN;
  private Vec2 blockedDirection = Vec2.ZERO;
  private double blindUntil = Double.NEGATIVE_INFINITY;

  public static final double BLIND_BACKOFF_SECONDS = 1.0;

  /**
   * @param rules pin limits from the game manual
   * @param field field geometry (robot size, walls)
   * @param safetyMargin start backing off this long before the limit (s)
   */
  public RulesGuard(PinRules rules, FieldConfig field, double safetyMargin) {
    this.pinTimer = new PinTimer(rules, field.robotHalfSize());
    this.field = field;
    this.safetyMargin = safetyMargin;
  }

  public static RulesGuard defaults(FieldConfig field) {
    return new RulesGuard(PinRules.placeholder(), field, 0.5);
  }

  /** Search radius (bumper gap, m) for the opponent blocking us when we detect a stall. */
  public static final double BLOCKED_SEARCH_GAP = 0.6;

  /** Update timers and return the (possibly overridden) target. Call every loop. */
  public Optional<DefenseTarget> apply(
      double nowSec, Pose self, List<TrackedRobot> opponents, Optional<DefenseTarget> target) {
    return apply(nowSec, self, opponents, target, false);
  }

  /**
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
    boolean blocked = isBlocked(commanded, actual);
    if (!blocked) {
      blockedSince = Double.NaN;
    } else if (Double.isNaN(blockedSince)) {
      blockedSince = nowSec;
      blockedDirection = commanded.normalized();
    }
    Optional<DefenseTarget> out = apply(nowSec, self, opponents, target, blocked);
    if (backingOff()) {
      blockedSince = Double.NaN; // the identified-opponent back-off is already handling it
      return out;
    }
    double trigger = pinTimer.rules().limitSeconds() - safetyMargin;
    if (!Double.isNaN(blockedSince) && nowSec - blockedSince >= trigger) {
      blindUntil = nowSec + BLIND_BACKOFF_SECONDS;
      blockedSince = Double.NaN;
      backoffsTriggered++;
    }
    if (nowSec < blindUntil && !blockedDirection.equals(Vec2.ZERO)) {
      Vec2 back = blockedDirection.times(-1.0);
      Vec2 retreat = field.clampInside(self.position().plus(back.times(STEP)));
      return Optional.of(new DefenseTarget(retreat, self.heading(), back.times(BACKOFF_SPEED), -1));
    }
    return out;
  }

  /** True while the blind (unidentified blocker) back-off is running. */
  public boolean blindBackingOff(double nowSec) {
    return nowSec < blindUntil;
  }

  /**
   * @param blocked true if our own sensors say we are pushing and not moving (see {@link
   *     #isBlocked})
   */
  public Optional<DefenseTarget> apply(
      double nowSec,
      Pose self,
      List<TrackedRobot> opponents,
      Optional<DefenseTarget> target,
      boolean blocked) {
    Map<Integer, Vec2> positions = new HashMap<>();
    Map<Integer, Vec2> velocities = new HashMap<>();
    for (TrackedRobot r : opponents) {
      positions.put(r.id(), r.positionAt(nowSec));
      velocities.put(r.id(), r.velocity());
    }
    int blocker = blocked ? nearestWithin(self.position(), positions) : -1;
    pinTimer.update(nowSec, self.position(), positions, id -> true, id -> id == blocker);

    // Finish back-offs whose episode ended, whose track was lost, or that timed out.
    backoffStarts
        .entrySet()
        .removeIf(
            e ->
                !positions.containsKey(e.getKey())
                    || !pinTimer.episodeOpen(e.getKey())
                    || nowSec - e.getValue() >= MAX_BACKOFF_SECONDS);

    // Start back-offs for every opponent close to the limit.
    double trigger = pinTimer.rules().limitSeconds() - safetyMargin;
    for (int id : positions.keySet()) {
      if (!backoffStarts.containsKey(id) && pinTimer.contactSeconds(id) >= trigger) {
        backoffStarts.put(id, nowSec);
        backoffsTriggered++;
      }
    }
    if (backoffStarts.isEmpty()) {
      return target;
    }

    Vec2 sum = Vec2.ZERO;
    int nearest = -1;
    double nearestDist = Double.POSITIVE_INFINITY;
    for (int id : backoffStarts.keySet()) {
      Vec2 opp = positions.get(id);
      sum = sum.plus(backoffDirection(self.position(), opp, velocities.get(id)));
      double d = opp.distanceTo(self.position());
      if (d < nearestDist) {
        nearestDist = d;
        nearest = id;
      }
    }
    Vec2 dir = sum.normalized();
    if (dir.equals(Vec2.ZERO)) {
      // Directions cancelled (squeezed from opposite sides): slide sideways.
      Vec2 any = self.position().minus(positions.get(nearest)).normalized();
      dir = new Vec2(-any.y(), any.x());
      if (dir.equals(Vec2.ZERO)) {
        dir = new Vec2(0.0, 1.0);
      }
    }
    Vec2 retreat = field.clampInside(self.position().plus(dir.times(STEP)));
    double heading = positions.get(nearest).minus(self.position()).angle(); // keep facing them
    return Optional.of(new DefenseTarget(retreat, heading, dir.times(BACKOFF_SPEED), nearest));
  }

  private int nearestWithin(Vec2 self, Map<Integer, Vec2> positions) {
    double limit = 2.0 * field.robotHalfSize() + BLOCKED_SEARCH_GAP;
    int best = -1;
    double bestDist = Double.POSITIVE_INFINITY;
    for (Map.Entry<Integer, Vec2> e : positions.entrySet()) {
      double d = e.getValue().distanceTo(self);
      if (d <= limit && d < bestDist) {
        best = e.getKey();
        bestDist = d;
      }
    }
    return best;
  }

  /**
   * Stall detection from our own odometry: we asked to move at least 1 m/s, but our actual speed
   * in that direction is under 30% of it.
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

  /** Unit direction to back off from one opponent (see class comment). */
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

  public int backoffsTriggered() {
    return backoffsTriggered;
  }
}
