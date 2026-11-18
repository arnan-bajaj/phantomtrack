package phantom.core.control;

/**
 * Shares control between the driver and the defense assist.
 *
 * <pre>
 *   overriding = |v_driver| > overrideFraction * maxSpeed
 *             or |omega_driver| > overrideFraction * maxOmega
 *   if the assist is off or the driver is overriding:  out = driver
 *   else:  v_out     = (1 - alpha) * v_driver + alpha * v_assist
 *          omega_out = (1 - alpha) * omega_driver + alpha * omega_assist
 * </pre>
 *
 * Hard override: if the assist is off, or the driver asks for more than {@code overrideFraction}
 * of full speed or of full turn rate, the driver gets full control. The driver can always take
 * over, by driving or by turning. overrideFraction acts as this class's deadband: below it the
 * driver's input gets weight 1 - alpha (none with alpha = 1). Because a non-overriding driver
 * command is within the limits, so is the blend.
 */
public final class DriverBlend {
  private final double alpha;
  private final double overrideFraction;

  /**
   * @param alpha 0..1 share given to the assist while it is engaged and the driver is not
   *     overriding (1 = assist fully drives)
   * @param overrideFraction 0..1 stick magnitude (as a fraction of max speed, or of max turn rate)
   *     that hands control back to the driver
   */
  public DriverBlend(double alpha, double overrideFraction) {
    if (!(alpha >= 0) || !(alpha <= 1)) {
      throw new IllegalArgumentException("alpha must be in [0, 1]");
    }
    if (!(overrideFraction >= 0) || !(overrideFraction <= 1)) {
      throw new IllegalArgumentException("overrideFraction must be in [0, 1]");
    }
    this.alpha = alpha;
    this.overrideFraction = overrideFraction;
  }

  public static DriverBlend defaults() {
    return new DriverBlend(1.0, 0.5);
  }

  /**
   * @param maxSpeed full speed (m/s): the override threshold is a fraction of it
   * @param maxOmega full turn rate (rad/s): the override threshold is a fraction of it
   */
  public ChassisCommand blend(
      ChassisCommand driver,
      ChassisCommand assist,
      boolean assistEngaged,
      double maxSpeed,
      double maxOmega) {
    if (!assistEngaged || assist == null || isOverriding(driver, maxSpeed, maxOmega)) {
      return driver;
    }
    return new ChassisCommand(
        driver.velocity().times(1.0 - alpha).plus(assist.velocity().times(alpha)),
        (1.0 - alpha) * driver.omega() + alpha * assist.omega());
  }

  /** True if the driver drives or turns past overrideFraction of full speed or turn rate. */
  public boolean isOverriding(ChassisCommand driver, double maxSpeed, double maxOmega) {
    return driver.velocity().norm() > overrideFraction * maxSpeed
        || Math.abs(driver.omega()) > overrideFraction * maxOmega;
  }
}
