package phantom.core.control;

import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseTarget;

/**
 * Turns a {@link DefenseTarget} into a field-relative chassis command.
 *
 * <pre>
 *   v_cmd = v_ff + kP * (p* - p_self),   capped at maxSpeed
 *   omega = kTheta * wrap(heading* - heading_self),   capped at maxOmega
 * </pre>
 *
 * Acceleration limits are the drivetrain's job (in sim: RobotBody).
 */
public final class HolonomicController {
  private final double kP;
  private final double kTheta;
  private final double maxSpeed;
  private final double maxOmega;

  public HolonomicController(double kP, double kTheta, double maxSpeed, double maxOmega) {
    this.kP = kP;
    this.kTheta = kTheta;
    this.maxSpeed = maxSpeed;
    this.maxOmega = maxOmega;
  }

  public static HolonomicController defaults() {
    return new HolonomicController(3.0, 5.0, 4.5, 2.0 * Math.PI);
  }

  public ChassisCommand calculate(Pose self, DefenseTarget target) {
    Vec2 error = target.position().minus(self.position());
    Vec2 v = target.velocityFeedforward().plus(error.times(kP)).clampNorm(maxSpeed);
    double headingError = Angles.difference(target.heading(), self.heading());
    double omega = Math.max(-maxOmega, Math.min(maxOmega, kTheta * headingError));
    return new ChassisCommand(v, omega);
  }

  public double maxSpeed() {
    return maxSpeed;
  }

  public double maxOmega() {
    return maxOmega;
  }
}
