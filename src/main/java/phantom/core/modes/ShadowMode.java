package phantom.core.modes;

import java.util.Optional;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Man-to-man "goal-side" defense: stay between the focus opponent and the goal they score at, a
 * fixed standoff distance in front of them, facing them.
 *
 * <pre>
 *   p_hat = p_opp + v_opp * lead                (predicted opponent position)
 *   u     = (goal - p_hat) / |goal - p_hat|     (unit vector, opponent -> goal)
 *   p*    = p_hat + min(d, |goal - p_hat|) * u   (never past the goal)
 * </pre>
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
    Vec2 predicted = opp.positionAt(ctx.timeSec() + leadSeconds);
    Vec2 goal = ctx.field().defendedGoal();

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
}
