package frc.robot.defense;

import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Vec2;

/**
 * The safety rules between the defense assist and the drivetrain. Two jobs:
 *
 * <p><b>1. Who drives this loop</b> ({@link #decide}). The assist drives only if ALL of these
 * hold, checked in this order:
 *
 * <pre>
 *   mode == TELEOP                         else NOT_TELEOP      (robot is commanded to stop)
 *   trigger > triggerThreshold             else OFF             (deadman: driver drives)
 *   no pose fault is latched               else POSE_FAULT      (driver drives)
 *   stickDeflection <= stickDeadband       else DRIVER_OVERRIDE (driver drives)
 *   robot is inside the test area          else OUTSIDE_AREA    (driver drives)
 * </pre>
 *
 * A pose fault latches: after the pose goes bad the assist stays off until the pose is healthy
 * again AND the driver has let go of the trigger. The robot never resumes on its own under a held
 * trigger.
 *
 * <p><b>2. Make an assist command safe</b> ({@link #filter}), in this order:
 *
 * <pre>
 *   speed cap  ->  acceleration limit (from the last command sent)  ->  geofence
 * </pre>
 *
 * The geofence is last so that nothing after it can put back motion toward a wall. These limits
 * apply to assist commands only. Whenever the driver drives, the driver's command is passed
 * through untouched.
 */
public final class SafetyGate {
  /** Who drives, and why. */
  public enum Decision {
    /** The assist drives (through {@link #filter}). */
    ASSIST,
    /** Deadman not held. */
    OFF,
    /** Driver moved a stick past the deadband while holding the deadman. */
    DRIVER_OVERRIDE,
    /** Disabled, autonomous or test mode. */
    NOT_TELEOP,
    /** Pose stale, frozen, jumped or NaN (latched until the trigger is released). */
    POSE_FAULT,
    /** Robot center is outside the test area. */
    OUTSIDE_AREA
  }

  private final SafetyConfig config;
  private final Geofence fence;
  private boolean faultLatched = false;

  /**
   * @param fence the test area; null means "no valid area", which keeps the assist off everywhere
   */
  public SafetyGate(SafetyConfig config, Geofence fence) {
    this.config = config;
    this.fence = fence;
  }

  /**
   * @param trigger right trigger, 0..1
   * @param stickDeflection largest raw stick deflection, 0..1 (translation magnitude or |turn|)
   * @param poseHealthy from {@link PoseMonitor#healthy()}
   * @param position robot center, blue frame (m)
   */
  public Decision decide(
      RobotMode mode, double trigger, double stickDeflection, boolean poseHealthy, Vec2 position) {
    boolean triggerHeld = trigger > config.triggerThreshold(); // false for NaN
    if (!poseHealthy) {
      faultLatched = true;
    } else if (!triggerHeld) {
      faultLatched = false;
    }
    if (mode != RobotMode.TELEOP) {
      return Decision.NOT_TELEOP;
    }
    if (!triggerHeld) {
      return Decision.OFF;
    }
    if (faultLatched) {
      return Decision.POSE_FAULT;
    }
    if (!(stickDeflection <= config.stickDeadband())) { // NaN counts as an override
      return Decision.DRIVER_OVERRIDE;
    }
    if (fence == null || position == null || !fence.contains(position)) {
      return Decision.OUTSIDE_AREA;
    }
    return Decision.ASSIST;
  }

  /**
   * @param assist what the defense code wants (blue frame)
   * @param previous the command sent to the drivetrain last loop
   * @param position robot center, blue frame (m)
   * @param dt seconds since the last loop
   * @return a command within the speed cap, the acceleration limit and the geofence; never NaN
   */
  public ChassisCommand filter(
      ChassisCommand assist, ChassisCommand previous, Vec2 position, double dt) {
    if (fence == null) {
      return ChassisCommand.STOP;
    }
    ChassisCommand capped = CommandLimits.cap(assist, config.speedCap(), config.omegaCap());
    ChassisCommand slewed =
        CommandLimits.limitAccel(
            previous, capped, config.accelLimit(), config.alphaLimit(), dt);
    // The slew starts from the last command, which may have been the driver's and faster than
    // the cap, so cap again before the fence.
    ChassisCommand recapped = CommandLimits.cap(slewed, config.speedCap(), config.omegaCap());
    return new ChassisCommand(fence.limit(position, recapped.velocity()), recapped.omega());
  }

  public boolean faultLatched() {
    return faultLatched;
  }

  public Geofence fence() {
    return fence;
  }

  public SafetyConfig config() {
    return config;
  }
}
