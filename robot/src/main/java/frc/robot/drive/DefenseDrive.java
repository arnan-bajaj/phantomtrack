package frc.robot.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * Everything PhantomTrack needs from a drivetrain, and nothing more. All values are in the real
 * field frame (WPILib: origin at the blue alliance wall corner), whichever alliance we are on;
 * PhantomTrack converts for red itself.
 *
 * <p>PhantomTrack does not do localization. {@link #getPose()} comes from the team's own pose
 * estimator (wheel odometry + whatever AprilTag pipeline the team uses). See robot/README.md for
 * CTRE Tuner X swerve and YAGSL adapters.
 */
public interface DefenseDrive {
  /** Current pose estimate (m, rad), field frame. */
  Pose2d getPose();

  /** Measured velocity, field-relative (m/s, rad/s). */
  ChassisSpeeds getFieldRelativeSpeeds();

  /** Drive at this field-relative velocity (m/s, rad/s) until the next call. */
  void driveFieldRelative(ChassisSpeeds speeds);

  /**
   * FPGA time (s, as {@link Timer#getFPGATimestamp()}) of the newest pose estimate. Used to switch
   * the assist off when the pose stops updating.
   *
   * <p>The default says "always fresh", which leaves only the frozen-pose check (pose not changing
   * while we command motion) to catch a dead estimator. Override it if your drivetrain reports
   * when its odometry last ran.
   */
  default double getPoseTimestampSeconds() {
    return Timer.getFPGATimestamp();
  }

  /**
   * Set the pose estimator to this pose (used to put the robot "on the field" for a test without
   * AprilTags). Return false if not supported.
   */
  default boolean resetPose(Pose2d pose) {
    return false;
  }
}
