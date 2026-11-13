package phantom.sim;

import phantom.core.geometry.Vec2;

/**
 * Pushing-match model for two robots in contact. Both robots are assumed equally strong (same
 * mass and traction), so the outcome depends only on who is driving into whom.
 *
 * <pre>
 *   n      = unit vector from A to B
 *   e_A    = clamp( vDesired_A . n    / vMax_A, -1, 1)   (+1 = A driving full speed into B)
 *   e_B    = clamp( vDesired_B . (-n) / vMax_B, -1, 1)   (+1 = B driving full speed into A)
 *   net    = e_A - e_B                                   (in [-2, 2])
 *   v_n    = pushSpeed * net / 2                         (shared velocity along n while touching)
 *   w_B    = 1/2 + net/4,  w_A = 1 - w_B                 (share of the overlap each one gives up)
 * </pre>
 *
 * Examples: both push full -> net 0 -> stall. A pushes full, B idle -> net 1 -> both slide toward
 * B at pushSpeed/2, B gives up 75% of the overlap. B bracing against an idle A -> A gets pushed.
 */
public final class Contact {
  private Contact() {}

  /** Speed of the pushed pair when one robot pushes full against a robot driving away (m/s). */
  public static final double PUSH_SPEED = 2.0;

  /** How hard a robot is driving toward the other along {@code towardOther}, in [-1, 1]. */
  public static double effort(Vec2 desiredVelocity, Vec2 towardOther, double maxSpeed) {
    double e = desiredVelocity.dot(towardOther) / maxSpeed;
    return Math.max(-1.0, Math.min(1.0, e));
  }

  /** Shared velocity along n (from A to B) while in contact. */
  public static double contactSpeed(double effortA, double effortB) {
    return PUSH_SPEED * (effortA - effortB) / 2.0;
  }

  /** Fraction of the overlap that B gives up (moves away); A gives up the rest. */
  public static double overlapShareB(double effortA, double effortB) {
    return 0.5 + (effortA - effortB) / 4.0;
  }
}
