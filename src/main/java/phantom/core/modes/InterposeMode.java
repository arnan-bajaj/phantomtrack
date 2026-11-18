package phantom.core.modes;

import java.util.Optional;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Lane block: stand on the opponent's straight path to the goal, a fraction lambda of the way
 * there, but never closer to them than minStandoff. Unlike Shadow, it takes space early instead of
 * retreating in front of the opponent.
 *
 * <pre>
 *   p_hat = p_opp + v_lead * lead,  v_lead = v_opp - min(0, v_opp . u0) u0,  u0 = unit(goal - p_opp)
 *           (no lead away from the goal, as in {@link ShadowMode}: with it, an opponent driving away
 *            had p* only minStandoff - |v| lead from its center)
 *   L     = |goal - p_hat|,  u = (goal - p_hat) / L
 *   dist  = min(max(lambda * L, minStandoff), L)
 *   p*    = p_hat + dist * u
 *   v_ff  = (1 - lambda) * v_opp      (derivative of (1-lambda) p_hat + lambda goal)
 * </pre>
 */
public final class InterposeMode implements DefenseMode {
  private final double lambda;
  private final double minStandoff;
  private final double leadSeconds;

  /**
   * @param lambda 0..1, how far along the opponent-to-goal line to stand (0 = on them, 1 = on goal)
   * @param minStandoff never closer than this to the opponent, center to center (m)
   * @param leadSeconds how far ahead to predict the opponent (s)
   */
  public InterposeMode(double lambda, double minStandoff, double leadSeconds) {
    if (lambda < 0 || lambda > 1) {
      throw new IllegalArgumentException("lambda must be in [0, 1]");
    }
    if (minStandoff <= 0) {
      throw new IllegalArgumentException("minStandoff must be > 0");
    }
    if (leadSeconds < 0) {
      throw new IllegalArgumentException("leadSeconds must be >= 0");
    }
    this.lambda = lambda;
    this.minStandoff = minStandoff;
    this.leadSeconds = leadSeconds;
  }

  public static InterposeMode defaults() {
    return new InterposeMode(0.5, 1.1, 0.25);
  }

  @Override
  public String name() {
    return "Interpose";
  }

  @Override
  public Optional<DefenseTarget> compute(DefenseContext ctx) {
    Optional<TrackedRobot> focus = ctx.focusOpponent();
    if (focus.isEmpty()) {
      return Optional.empty();
    }
    TrackedRobot opp = focus.get();
    Vec2 predicted =
        ShadowMode.predictTowardGoal(opp, ctx.timeSec(), leadSeconds, ctx.field().defendedGoal());
    Vec2 toGoal = ctx.field().defendedGoal().minus(predicted);
    double length = toGoal.norm();
    Vec2 u = toGoal.normalized();
    if (u.equals(Vec2.ZERO)) {
      u = ctx.self().position().minus(predicted).normalized();
    }

    double dist = Math.min(Math.max(lambda * length, minStandoff), length);
    Vec2 target = ctx.field().clampInside(predicted.plus(u.times(dist)));
    double heading = predicted.minus(target).angle();
    Vec2 ff = opp.velocity().times(1.0 - lambda);
    return Optional.of(new DefenseTarget(target, heading, ff, opp.id()));
  }
}
