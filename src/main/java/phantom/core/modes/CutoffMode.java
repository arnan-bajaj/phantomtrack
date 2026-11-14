package phantom.core.modes;

import java.util.List;
import java.util.Optional;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Vec2;
import phantom.core.prediction.IntentPredictor;
import phantom.core.tracking.TrackedRobot;

/**
 * Cut-off: instead of reacting to where the opponent is, go to where it is going to be. Predict
 * its route to the goal ({@link IntentPredictor}) and stand on the first point of that route we
 * can reach before it does.
 *
 * <pre>
 * Heading to the goal:
 *   R = route(p_opp -> goal), L = length(R), s = max(|v_opp|, MIN_OPP_SPEED)
 *   a* = smallest a in {standoff, standoff + STEP, ...} <= L - GOAL_GAP with
 *          |p_self - R(a)| / ourSpeed + margin  <=  a / s       (we get there first)
 *   target = R(a*), or R(L - GOAL_GAP) if no such a
 * Heading anywhere else (back to a pickup):
 *   ambush on its next goal run: R' = route(pickup -> goal),
 *   target = R'(length(R') - AMBUSH)   (wait AMBUSH meters before the goal)
 * Face the opponent; no velocity feedforward (we wait for them).
 * </pre>
 */
public final class CutoffMode implements DefenseMode {
  public static final double STEP = 0.25;
  public static final double MIN_OPP_SPEED = 2.0;
  public static final double GOAL_GAP = 0.5;
  public static final double AMBUSH = 3.0;

  private final double standoff;
  private final double ourSpeed;
  private final double margin;

  /**
   * @param standoff closest point on their route to consider, measured from them (m)
   * @param ourSpeed our top speed (m/s)
   * @param margin how much earlier than them we want to arrive (s)
   */
  public CutoffMode(double standoff, double ourSpeed, double margin) {
    if (standoff <= 0 || ourSpeed <= 0 || margin < 0) {
      throw new IllegalArgumentException("invalid CutoffMode parameters");
    }
    this.standoff = standoff;
    this.ourSpeed = ourSpeed;
    this.margin = margin;
  }

  public static CutoffMode defaults() {
    return new CutoffMode(1.1, 4.5, 0.3);
  }

  @Override
  public String name() {
    return "Cut-off";
  }

  @Override
  public Optional<DefenseTarget> compute(DefenseContext ctx) {
    Optional<TrackedRobot> focus = ctx.focusOpponent();
    if (focus.isEmpty()) {
      return Optional.empty();
    }
    TrackedRobot opp = focus.get();
    FieldConfig field = ctx.field();
    Vec2 goal = field.defendedGoal();
    Vec2 p = opp.positionAt(ctx.timeSec());
    Vec2 self = ctx.self().position();
    Vec2 dest = IntentPredictor.destination(opp, ctx.timeSec(), field);

    Vec2 target;
    if (dest.equals(goal)) {
      List<Vec2> route = IntentPredictor.route(p, goal, field);
      double length = IntentPredictor.length(route);
      double oppSpeed = Math.max(opp.velocity().norm(), MIN_OPP_SPEED);
      double last = Math.max(0.0, length - GOAL_GAP);
      target = IntentPredictor.pointAlong(route, last);
      for (double a = standoff; a <= last + 1e-9; a += STEP) {
        Vec2 q = IntentPredictor.pointAlong(route, a);
        if (self.distanceTo(q) / ourSpeed + margin <= a / oppSpeed) {
          target = q;
          break;
        }
      }
    } else {
      List<Vec2> next = IntentPredictor.route(dest, goal, field);
      target = IntentPredictor.pointAlong(next, Math.max(0.0, IntentPredictor.length(next) - AMBUSH));
    }
    target = field.clampInside(target);
    Vec2 facing = p.minus(target);
    double heading = facing.equals(Vec2.ZERO) ? 0.0 : facing.angle();
    return Optional.of(new DefenseTarget(target, heading, Vec2.ZERO, opp.id()));
  }
}
