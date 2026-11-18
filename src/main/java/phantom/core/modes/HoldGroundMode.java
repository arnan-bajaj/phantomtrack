package phantom.core.modes;

import java.util.Optional;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Wraps any mode so it stops giving ground on contact. The inner mode decides where to stand; once
 * the opponent it is reacting to is within {@code holdGap} (bumper to bumper) AND we are goal-side
 * of it, we push straight back into it instead of retreating.
 *
 * <pre>
 *   gap      = |p_opp - p_self| - 2 * robotHalfSize
 *   hold if  gap <= holdGap  and  |p_self - goal| < |p_opp - goal|
 *   n        = unit(p_opp - p_self)
 *   p*       = clampInside(p_self + 0.3 * n)   (like every mode's target: pushing around a wall
 *                                              corner, p_self + 0.3 n can be inside the wall)
 *   v_ff     = pushSpeed * n          (drive into them at full effort so a pushing match stalls)
 *   heading  = angle(n)               (face them)
 * </pre>
 *
 * Still a pure function of the context. Pin limits are enforced separately by RulesGuard.
 */
public final class HoldGroundMode implements DefenseMode {
  private static final double STEP = 0.3;

  private final DefenseMode inner;
  private final double holdGap;
  private final double pushSpeed;

  public HoldGroundMode(DefenseMode inner, double holdGap, double pushSpeed) {
    if (holdGap < 0 || pushSpeed <= 0) {
      throw new IllegalArgumentException("holdGap must be >= 0 and pushSpeed > 0");
    }
    this.inner = inner;
    this.holdGap = holdGap;
    this.pushSpeed = pushSpeed;
  }

  public static HoldGroundMode wrap(DefenseMode inner) {
    // 0.3 m is bigger than the 0.2 m gap Shadow/Interpose keep (1.1 m center-to-center), so the
    // hold kicks in as soon as an opponent closes inside the standoff.
    return new HoldGroundMode(inner, 0.3, 4.5);
  }

  @Override
  public String name() {
    return inner.name() + " + hold";
  }

  public DefenseMode inner() {
    return inner;
  }

  @Override
  public Optional<DefenseTarget> compute(DefenseContext ctx) {
    Optional<DefenseTarget> base = inner.compute(ctx);
    Optional<DefenseTarget> hold = holdTarget(ctx, base);
    return hold.isPresent() ? hold : base;
  }

  /**
   * The push target if the hold rule fires for the inner mode's target {@code base}, otherwise
   * empty (then {@link #compute} returns {@code base}). Lets the caller tell whether we are
   * holding without computing the inner mode twice.
   */
  public Optional<DefenseTarget> holdTarget(DefenseContext ctx, Optional<DefenseTarget> base) {
    if (base.isEmpty() || base.get().opponentId() < 0) {
      return Optional.empty();
    }
    TrackedRobot opp = null;
    for (TrackedRobot r : ctx.opponents()) {
      if (r.id() == base.get().opponentId()) {
        opp = r;
      }
    }
    if (opp == null) {
      return Optional.empty();
    }
    Vec2 self = ctx.self().position();
    Vec2 oppPos = opp.positionAt(ctx.timeSec());
    Vec2 goal = ctx.field().defendedGoal();
    double gap = oppPos.distanceTo(self) - 2.0 * ctx.field().robotHalfSize();
    boolean goalSide = self.distanceTo(goal) < oppPos.distanceTo(goal);
    if (gap > holdGap || !goalSide) {
      return Optional.empty();
    }
    Vec2 n = oppPos.minus(self).normalized();
    if (n.equals(Vec2.ZERO)) {
      return Optional.empty();
    }
    return Optional.of(
        new DefenseTarget(
            ctx.field().clampInside(self.plus(n.times(STEP))),
            n.angle(),
            n.times(pushSpeed),
            opp.id()));
  }
}
