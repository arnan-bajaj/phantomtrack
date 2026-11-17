package frc.robot.defense;

import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Vec2;

/**
 * Speed cap and acceleration limit for assist commands. Pure functions.
 *
 * <pre>
 *   cap:    v_out     = v * min(1, speedCap / |v|)             (same direction, shorter)
 *           omega_out = clamp(omega, -omegaCap, +omegaCap)
 *
 *   accel:  v_out     = v_prev + clampNorm(v_want - v_prev, maxAccel * dt)
 *           omega_out = omega_prev + clamp(omega_want - omega_prev, -maxAlpha * dt, +maxAlpha * dt)
 * </pre>
 *
 * A command with NaN or infinity in it becomes STOP, so bad math upstream can never reach the
 * drivetrain.
 */
public final class CommandLimits {
  private CommandLimits() {}

  public static boolean isFinite(ChassisCommand c) {
    return c != null && c.velocity().isFinite() && Double.isFinite(c.omega());
  }

  /** STOP if the command is null or not finite, otherwise the command itself. */
  public static ChassisCommand sanitize(ChassisCommand c) {
    return isFinite(c) ? c : ChassisCommand.STOP;
  }

  public static ChassisCommand cap(ChassisCommand c, double speedCap, double omegaCap) {
    if (!isFinite(c)) {
      return ChassisCommand.STOP;
    }
    return new ChassisCommand(
        c.velocity().clampNorm(Math.max(0.0, speedCap)),
        Math.max(-omegaCap, Math.min(omegaCap, c.omega())));
  }

  /**
   * @param previous the command that was sent last loop
   * @param want the command we would like to send now
   * @param maxAccel m/s^2
   * @param maxAlpha rad/s^2
   * @param dt seconds since the last loop (0 means "no change allowed")
   */
  public static ChassisCommand limitAccel(
      ChassisCommand previous, ChassisCommand want, double maxAccel, double maxAlpha, double dt) {
    if (!isFinite(want)) {
      return ChassisCommand.STOP;
    }
    ChassisCommand prev = sanitize(previous);
    double step = Math.max(0.0, dt);
    Vec2 dv = want.velocity().minus(prev.velocity()).clampNorm(maxAccel * step);
    double maxDw = maxAlpha * step;
    double dw = Math.max(-maxDw, Math.min(maxDw, want.omega() - prev.omega()));
    return new ChassisCommand(prev.velocity().plus(dv), prev.omega() + dw);
  }
}
