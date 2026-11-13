package phantom.core.control;

/**
 * Shares control between the driver and the defense assist.
 *
 * <pre>
 *   v_out = (1 - alpha) * v_driver + alpha * v_assist
 * </pre>
 *
 * Hard override: if the assist is off, or the driver pushes the stick past {@code
 * overrideFraction} of full speed, the driver gets full control. The driver can always take over.
 */
public final class DriverBlend {
  private final double alpha;
  private final double overrideFraction;

  /**
   * @param alpha 0..1 share given to the assist while it is engaged and the driver is not
   *     overriding (1 = assist fully drives)
   * @param overrideFraction 0..1 stick magnitude (as a fraction of max speed) that hands control
   *     back to the driver
   */
  public DriverBlend(double alpha, double overrideFraction) {
    if (alpha < 0 || alpha > 1) {
      throw new IllegalArgumentException("alpha must be in [0, 1]");
    }
    this.alpha = alpha;
    this.overrideFraction = overrideFraction;
  }

  public static DriverBlend defaults() {
    return new DriverBlend(1.0, 0.5);
  }

  public ChassisCommand blend(
      ChassisCommand driver, ChassisCommand assist, boolean assistEngaged, double maxSpeed) {
    if (!assistEngaged || assist == null || isOverriding(driver, maxSpeed)) {
      return driver;
    }
    return new ChassisCommand(
        driver.velocity().times(1.0 - alpha).plus(assist.velocity().times(alpha)),
        (1.0 - alpha) * driver.omega() + alpha * assist.omega());
  }

  public boolean isOverriding(ChassisCommand driver, double maxSpeed) {
    return driver.velocity().norm() > overrideFraction * maxSpeed;
  }
}
