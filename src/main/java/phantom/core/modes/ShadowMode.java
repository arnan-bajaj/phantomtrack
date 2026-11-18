package phantom.core.modes;

import java.util.Optional;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Man-to-man "goal-side" defense: stay between the focus opponent and the goal they score at, a
 * fixed standoff distance in front of them, facing them.
 *
 * <pre>
 *   u0     = unit(goal - p_opp)
 *   v_lead = v_opp - min(0, v_opp . u0) * u0     (the lead never moves them away from the goal)
 *   p_hat  = p_opp + v_lead * lead               (predicted opponent position)
 *   u      = (goal - p_hat) / |goal - p_hat|     (unit vector, opponent -> goal)
 *   p*     = p_hat + min(d, |goal - p_hat|) * u   (never past the goal)
 *   v_ff   = v_opp
 * </pre>
 *
 * Why no lead away from the goal: following an opponent that drives away from the goal, the full
 * lead put p* only |d - |v| lead| from its center (0.025 m at 4.5 m/s), inside its bumpers, so
 * the robot rammed its back. Without that part p* stays d in front of it (audit M1).
 */
public final class ShadowMode implements DefenseMode {
  private final double standoffMeters;
  private final double leadSeconds;

  /**
   * @param standoffMeters center-to-center distance to hold from the opponent. Make it a bit more
   *     than two robot half-sizes so we are not in constant contact.
   * @param leadSeconds how far ahead to predict the opponent (s); covers our reaction lag
   */
  public ShadowMode(double standoffMeters, double leadSeconds) {
    if (standoffMeters <= 0) {
      throw new IllegalArgumentException("standoffMeters must be > 0");
    }
    if (leadSeconds < 0) {
      throw new IllegalArgumentException("leadSeconds must be >= 0");
    }
    this.standoffMeters = standoffMeters;
    this.leadSeconds = leadSeconds;
  }

  public static ShadowMode defaults() {
    return new ShadowMode(1.1, 0.25);
  }

  @Override
  public String name() {
    return "Shadow";
  }

  @Override
  public Optional<DefenseTarget> compute(DefenseContext ctx) {
    Optional<TrackedRobot> focus = ctx.focusOpponent();
    if (focus.isEmpty()) {
      return Optional.empty();
    }
    TrackedRobot opp = focus.get();
    Vec2 goal = ctx.field().defendedGoal();
    Vec2 predicted = predictTowardGoal(opp, ctx.timeSec(), leadSeconds, goal);

    Vec2 toGoal = goal.minus(predicted);
    double distToGoal = toGoal.norm();
    Vec2 u = toGoal.normalized();
    if (u.equals(Vec2.ZERO)) {
      // Opponent is on top of the goal: no defined goal-side direction. Hold our ground between
      // them and wherever we already are.
      u = ctx.self().position().minus(predicted).normalized();
    }

    Vec2 target = predicted.plus(u.times(Math.min(standoffMeters, distToGoal)));
    target = ctx.field().clampInside(target);
    double heading = predicted.minus(target).angle(); // face the opponent

    return Optional.of(new DefenseTarget(target, heading, opp.velocity(), opp.id()));
  }

  /**
   * Where the opponent will be after {@code leadSeconds}, leading only the part of its velocity
   * that does not point away from the goal (see the class comment):
   *
   * <pre>
   *   p_hat = p_opp(now) + (v - min(0, v . u0) * u0) * lead,   u0 = unit(goal - p_opp(now))
   * </pre>
   *
   * With the opponent on the goal (u0 = ZERO) it is the plain constant-velocity lead.
   */
  static Vec2 predictTowardGoal(TrackedRobot opp, double nowSec, double leadSeconds, Vec2 goal) {
    Vec2 now = opp.positionAt(nowSec);
    Vec2 u0 = goal.minus(now).normalized();
    Vec2 v = opp.velocity();
    Vec2 lead = v.minus(u0.times(Math.min(0.0, v.dot(u0))));
    return now.plus(lead.times(leadSeconds));
  }
}
