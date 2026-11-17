package phantom.core.rules;

import phantom.core.geometry.Vec2;

/**
 * What the drivetrain can tell us about itself in one 20 ms loop. On the robot these come from the
 * swerve module requests, wheel odometry, the IMU and the drive motor controllers; in the sim from
 * {@code phantom.sim.DriveTelemetrySim}.
 *
 * @param timestampSec time the sample is valid for (s)
 * @param commandedVelocity field velocity we asked the chassis for (m/s)
 * @param measuredVelocity field velocity the chassis actually has (m/s)
 * @param measuredAcceleration field acceleration of the chassis (m/s^2)
 * @param driveCurrentAmps mean of |stator current| over the drive motors (A); NaN if not available
 */
public record DriveTelemetry(
    double timestampSec,
    Vec2 commandedVelocity,
    Vec2 measuredVelocity,
    Vec2 measuredAcceleration,
    double driveCurrentAmps) {

  /** A robot that is asked to do nothing and does nothing. */
  public static DriveTelemetry idle(double timestampSec) {
    return new DriveTelemetry(timestampSec, Vec2.ZERO, Vec2.ZERO, Vec2.ZERO, 0.0);
  }

  /**
   * For callers that only know velocities (no IMU, no current reading). Acceleration is taken from
   * two velocity samples:
   *
   * <pre>
   *   a = (v_now - v_previous) / dt        (ZERO if dt <= 0)
   * </pre>
   */
  public static DriveTelemetry fromVelocities(
      double timestampSec, Vec2 commanded, Vec2 measured, Vec2 previousMeasured, double dt) {
    Vec2 accel = dt > 0.0 ? measured.minus(previousMeasured).times(1.0 / dt) : Vec2.ZERO;
    return new DriveTelemetry(timestampSec, commanded, measured, accel, Double.NaN);
  }

  /** False if the drive current was not measured (NaN). */
  public boolean hasCurrent() {
    return Double.isFinite(driveCurrentAmps);
  }
}
