package frc.robot.defense;

import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Vec2;

/**
 * Turns raw stick values into the driver's field-relative command, in the blue frame (so "stick
 * forward" is +x; the alliance adapter flips it for red, like normal field-oriented driving).
 *
 * <pre>
 *   translation (radial deadband, so diagonals are not squeezed):
 *     m = |(x, y)|
 *     m <= deadband:  v = 0
 *     else:           v = (x, y) / m * (min(m, 1) - deadband) / (1 - deadband) * maxSpeed
 *   rotation:
 *     |r| <= deadband:  omega = 0
 *     else:             omega = sign(r) * (min(|r|, 1) - deadband) / (1 - deadband) * maxOmega
 *   deflection = max(m, |r|)      (raw, before the deadband: what SafetyGate calls an override)
 * </pre>
 *
 * NaN from a broken controller reads as "no input" for the command but as a full override for
 * {@link #deflection}, so the assist lets go.
 */
public final class DriverInput {
  private DriverInput() {}

  /**
   * @param forward stick forward, -1..1 (Xbox: -leftY)
   * @param left stick left, -1..1 (Xbox: -leftX)
   * @param ccw turn counter-clockwise, -1..1 (Xbox: -rightX)
   */
  public static ChassisCommand command(
      double forward, double left, double ccw, double deadband, double maxSpeed, double maxOmega) {
    Vec2 stick = new Vec2(forward, left);
    Vec2 v = Vec2.ZERO;
    double m = stick.norm();
    if (stick.isFinite() && m > deadband) {
      double scaled = (Math.min(m, 1.0) - deadband) / (1.0 - deadband);
      v = stick.times(scaled * maxSpeed / m);
    }
    double omega = 0.0;
    if (Double.isFinite(ccw) && Math.abs(ccw) > deadband) {
      omega = Math.signum(ccw) * (Math.min(Math.abs(ccw), 1.0) - deadband) / (1.0 - deadband) * maxOmega;
    }
    return new ChassisCommand(v, omega);
  }

  /** Largest raw stick deflection, 0..1+; NaN if any input is NaN. */
  public static double deflection(double forward, double left, double ccw) {
    return Math.max(Math.hypot(forward, left), Math.abs(ccw));
  }
}
