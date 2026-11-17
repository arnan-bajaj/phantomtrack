package phantom.sim;

/**
 * Every constant of the simulated swerve drive in one place: the drive motor, the gearing, the
 * wheel, the electrical limits and the velocity loop. Used only to estimate drive current in the
 * sim; the real robot reads current from its motor controllers.
 *
 * <pre>
 *   R      = nominalVolts / stallCurrentAmps                      (winding resistance, ohm)
 *   Kv     = freeSpeedRadPerSec / (nominalVolts - R * freeCurrentAmps)   (rad/s per volt)
 *   omega  = wheelSpeed / wheelRadiusM * gearRatio                (motor speed, rad/s)
 *   V      = clamp(omega_cmd / Kv + velocityKp * (omega_cmd - omega), -batteryVolts, +batteryVolts)
 *   I      = clamp((V - omega / Kv) / R, -currentLimitAmps, +currentLimitAmps)
 * </pre>
 *
 * The first term of V is the feedforward that holds the commanded speed; the second is the
 * velocity loop reacting to the speed error. A stalled wheel (omega = 0, omega_cmd large) saturates
 * V at the battery and I at the current limit.
 *
 * @param name label for reports
 * @param nominalVolts voltage the datasheet numbers are quoted at (V)
 * @param freeSpeedRadPerSec motor speed with no load at nominal voltage (rad/s)
 * @param stallCurrentAmps current at zero speed and nominal voltage (A)
 * @param freeCurrentAmps current at free speed (A)
 * @param gearRatio motor turns per wheel turn
 * @param wheelRadiusM wheel radius (m)
 * @param moduleOffsetM distance of each swerve module from the robot center along x and along y (m)
 * @param batteryVolts the most the controller can apply (V)
 * @param currentLimitAmps stator current limit set in the motor controller (A)
 * @param velocityKp velocity-loop gain (V per rad/s of motor speed error)
 */
public record DriveMotorModel(
    String name,
    double nominalVolts,
    double freeSpeedRadPerSec,
    double stallCurrentAmps,
    double freeCurrentAmps,
    double gearRatio,
    double wheelRadiusM,
    double moduleOffsetM,
    double batteryVolts,
    double currentLimitAmps,
    double velocityKp) {

  private static final double RPM = 2.0 * Math.PI / 60.0;

  /**
   * Kraken X60 (trapezoidal commutation) on an SDS MK4i L2 module. Motor numbers are the ones
   * WPILib's {@code DCMotor.getKrakenX60} uses: 12 V, 6000 rpm free, 366 A stall, 2 A free.
   * Gearing 6.75:1 and 4 in wheels (0.0508 m radius) give 6000 rpm -> 4.73 m/s free speed.
   *
   * <p>PLACEHOLDERS until the real robot is known: module offset 0.27 m, 60 A stator limit, and
   * velocityKp 0.016 V per rad/s (0.1 V per rotation/s, the CTRE swerve template value).
   */
  public static DriveMotorModel krakenX60() {
    return new DriveMotorModel(
        "Kraken X60", 12.0, 6000.0 * RPM, 366.0, 2.0, 6.75, 0.0508, 0.27, 12.0, 60.0, 0.016);
  }

  /**
   * REV NEO on the same module, for teams without Krakens. Motor numbers are the ones WPILib's
   * {@code DCMotor.getNEO} uses: 12 V, 5676 rpm free, 105 A stall, 1.8 A free. PLACEHOLDERS: 40 A
   * current limit, same gearing and gain as the Kraken setup.
   */
  public static DriveMotorModel neo() {
    return new DriveMotorModel(
        "NEO", 12.0, 5676.0 * RPM, 105.0, 1.8, 6.75, 0.0508, 0.27, 12.0, 40.0, 0.016);
  }

  /** Winding resistance R (ohm). */
  public double resistanceOhms() {
    return nominalVolts / stallCurrentAmps;
  }

  /** Speed constant Kv (rad/s per volt). */
  public double kvRadPerSecPerVolt() {
    return freeSpeedRadPerSec / (nominalVolts - resistanceOhms() * freeCurrentAmps);
  }

  /** Motor speed (rad/s) for a wheel rolling at {@code wheelSpeed} (m/s). */
  public double motorSpeed(double wheelSpeed) {
    return wheelSpeed / wheelRadiusM * gearRatio;
  }

  /** Voltage the velocity loop applies (V), limited by the battery. See the class comment. */
  public double appliedVolts(double commandedMotorSpeed, double measuredMotorSpeed) {
    double v =
        commandedMotorSpeed / kvRadPerSecPerVolt()
            + velocityKp * (commandedMotorSpeed - measuredMotorSpeed);
    return Math.max(-batteryVolts, Math.min(batteryVolts, v));
  }

  /** Stator current (A) at this voltage and motor speed, limited by the current limit. */
  public double current(double volts, double measuredMotorSpeed) {
    double i = (volts - measuredMotorSpeed / kvRadPerSecPerVolt()) / resistanceOhms();
    return Math.max(-currentLimitAmps, Math.min(currentLimitAmps, i));
  }

  /** Stator current (A) of one module asked for one wheel speed while rolling at another (m/s). */
  public double moduleCurrent(double commandedWheelSpeed, double measuredWheelSpeed) {
    double cmd = motorSpeed(commandedWheelSpeed);
    double meas = motorSpeed(measuredWheelSpeed);
    return current(appliedVolts(cmd, meas), meas);
  }
}
