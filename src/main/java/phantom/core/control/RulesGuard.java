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
 *
 *   per opponent i being backed off from, step ASIDE so a pushing opponent can pass:
 *     if |v_i| > 0.3:  d_i = perp(unit(v_i)), sign chosen so d_i . (p_self - p_i) >= 0
 *     else:            d_i = unit(p_self - p_i)                        (straight away)
 *   D    = unit(sum of d_i)          (several at once, e.g. when sandwiched)
 *   p*   = p_self + STEP * D,  v_ff = BACKOFF_SPEED * D
 *
 *   stop backing off from i when its pin episode ends (gap >= resetSeparation)
 *   or MAX_BACKOFF_SECONDS pass (e.g. a wall stops us)
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

  /** Update timers and return the (possibly overridden) target. Call every loop. */
  public Optional<DefenseTarget> apply(
      double nowSec, Pose self, List<TrackedRobot> opponents, Optional<DefenseTarget> target) {
    Map<Integer, Vec2> positions = new HashMap<>();
    Map<Integer, Vec2> velocities = new HashMap<>();
    for (TrackedRobot r : opponents) {
      positions.put(r.id(), r.positionAt(nowSec));
      velocities.put(r.id(), r.velocity());
    }
    pinTimer.update(nowSec, self.position(), positions);

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
