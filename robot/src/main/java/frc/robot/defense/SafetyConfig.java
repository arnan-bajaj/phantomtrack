package frc.robot.defense;

/**
 * Every number the safety rules use, in one place. {@link #firstTest()} is what the robot boots
 * with; the dashboard can change the first seven (see {@code frc.robot.DefenseDashboard}).
 *
 * @param speedCap fastest the ASSIST may drive (m/s)
 * @param omegaCap fastest the assist may turn (rad/s)
 * @param accelLimit how fast an assist command may change (m/s^2); also the braking rate assumed
 *     by the geofence
 * @param areaMinX west edge of the test area, blue frame (m)
 * @param areaMinY south edge (m)
 * @param areaMaxX east edge (m)
 * @param areaMaxY north edge (m)
 * @param fenceMargin the assist never drives toward an edge once the robot center is this close to
 *     it (m)
 * @param alphaLimit how fast the assist's turn rate may change (rad/s^2)
 * @param triggerThreshold right-trigger value above which the deadman counts as held (0..1)
 * @param stickDeadband stick deflection above which the driver is overriding (0..1)
 * @param maxPoseAgeSec pose older than this is stale (s)
 * @param maxPoseJumpMeters pose moving more than this in one loop is a jump (m)
 * @param maxHeadingJumpRad heading changing more than this in one loop is a jump (rad)
 * @param frozenPoseSec pose identical for this long while we command motion is frozen (s)
 * @param poseRecoverySec the pose must look fine for this long before it is trusted again (s)
 * @param pinWarningSec contact time at which the controller starts to rumble (s)
 */
public record SafetyConfig(
    double speedCap,
    double omegaCap,
    double accelLimit,
    double areaMinX,
    double areaMinY,
    double areaMaxX,
    double areaMaxY,
    double fenceMargin,
    double alphaLimit,
    double triggerThreshold,
    double stickDeadband,
    double maxPoseAgeSec,
    double maxPoseJumpMeters,
    double maxHeadingJumpRad,
    double frozenPoseSec,
    double poseRecoverySec,
    double pinWarningSec) {

  /** Hard ceiling for the dashboard speed cap: a typical swerve's top speed (m/s). */
  public static final double MAX_SPEED_CAP = 4.5;

  /**
   * PLACEHOLDER. Time from a command leaving the assist to the wheels acting on it, beyond the
   * 20 ms loop (s): CAN, motor-controller control loop, module steering. Used for the pin guard's
   * margin. Measure it on the real drivetrain (command step vs measured velocity in the log).
   */
  public static final double ASSUMED_COMMAND_LATENCY_SEC = 0.1;

  /** Slowest useful cap (m/s); below this the robot barely overcomes carpet friction. */
  public static final double MIN_SPEED_CAP = 0.1;

  /**
   * Settings for a first test on carpet. The test area is a 5.0 x 5.0 m box in front of the
   * defended goal of the placeholder field (goal at x = 13.0, y = 4.035), which is where every
   * defense mode does its work. A robot whose pose has never been set (odometry starts at 0, 0)
   * is far outside it, so the assist refuses to run until the pose is real.
   */
  public static SafetyConfig firstTest() {
    return new SafetyConfig(
        1.0, // speedCap m/s
        2.0, // omegaCap rad/s
        2.0, // accelLimit m/s^2
        7.5, // areaMinX
        1.5, // areaMinY
        12.5, // areaMaxX
        6.5, // areaMaxY
        0.75, // fenceMargin m
        6.0, // alphaLimit rad/s^2
        0.5, // triggerThreshold
        0.1, // stickDeadband
        0.25, // maxPoseAgeSec
        0.5, // maxPoseJumpMeters
        0.5, // maxHeadingJumpRad (25 rad/s at 50 Hz: no FRC robot turns that fast)
        0.5, // frozenPoseSec
        0.5, // poseRecoverySec
        1.5); // pinWarningSec (the pin guard backs off at 2.5 s, the placeholder limit is 3.0 s)
  }

  /**
   * A copy with the dashboard-tunable values replaced. Out-of-range numbers are pulled back into
   * range rather than trusted: speed cap into [MIN_SPEED_CAP, MAX_SPEED_CAP], accel limit into
   * [0.5, 10], margin into [0, 5]. NaN falls back to this config's value.
   */
  public SafetyConfig withTunables(
      double newSpeedCap,
      double newAccelLimit,
      double minX,
      double minY,
      double maxX,
      double maxY,
      double newMargin) {
    return new SafetyConfig(
        clamp(newSpeedCap, MIN_SPEED_CAP, MAX_SPEED_CAP, speedCap),
        omegaCap,
        clamp(newAccelLimit, 0.5, 10.0, accelLimit),
        Double.isFinite(minX) ? minX : areaMinX,
        Double.isFinite(minY) ? minY : areaMinY,
        Double.isFinite(maxX) ? maxX : areaMaxX,
        Double.isFinite(maxY) ? maxY : areaMaxY,
        clamp(newMargin, 0.0, 5.0, fenceMargin),
        alphaLimit,
        triggerThreshold,
        stickDeadband,
        maxPoseAgeSec,
        maxPoseJumpMeters,
        maxHeadingJumpRad,
        frozenPoseSec,
        poseRecoverySec,
        pinWarningSec);
  }

  private static double clamp(double v, double lo, double hi, double fallback) {
    return Double.isFinite(v) ? Math.max(lo, Math.min(hi, v)) : fallback;
  }
}
