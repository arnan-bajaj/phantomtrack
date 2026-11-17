package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.defense.DriverInput;
import frc.robot.defense.PhantomTrack;
import frc.robot.defense.RobotMode;
import frc.robot.drive.DefenseDrive;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/**
 * The drivetrain's default command: one PhantomTrack loop every 20 ms.
 *
 * <pre>
 *   controller + drivetrain (real frame)
 *     -> AllianceFrame.toBlue -> PhantomTrack.update (all safety rules, blue frame)
 *     -> AllianceFrame.commandToReal -> drivetrain.driveFieldRelative
 *   then rumble + telemetry
 * </pre>
 *
 * Deliberately thin: every decision is in frc.robot.defense, which has no WPILib and is unit
 * tested. Runs while disabled too (PhantomTrack outputs STOP then), so telemetry and the pose
 * monitor stay live.
 */
public final class DefenseTeleop extends Command {
  private final DefenseDrive drive;
  private final XboxController hid;
  private final PhantomTrack pt;
  private final AllianceFrame frame;
  private final DefenseDashboard dashboard;
  private final DefenseTelemetry telemetry;

  public DefenseTeleop(
      DefenseDrive drive,
      Subsystem driveSubsystem,
      XboxController hid,
      PhantomTrack pt,
      AllianceFrame frame,
      DefenseDashboard dashboard,
      DefenseTelemetry telemetry) {
    this.drive = drive;
    this.hid = hid;
    this.pt = pt;
    this.frame = frame;
    this.dashboard = dashboard;
    this.telemetry = telemetry;
    addRequirements(driveSubsystem);
  }

  /** The Driver Station state as PhantomTrack sees it. Only TELEOP can ever assist. */
  public static RobotMode mode() {
    if (DriverStation.isDisabled()) {
      return RobotMode.DISABLED;
    }
    if (DriverStation.isAutonomous()) {
      return RobotMode.AUTONOMOUS;
    }
    if (DriverStation.isTest()) {
      return RobotMode.TEST;
    }
    return DriverStation.isTeleop() ? RobotMode.TELEOP : RobotMode.DISABLED;
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    DefenseDashboard.Settings settings = dashboard.read();
    pt.requestSettings(settings.safety(), settings.ghosts());
    if (settings.resetGhosts()) {
      pt.requestGhostReset();
    }

    boolean red = AllianceFrame.isRedNow();
    Pose2d realPose = drive.getPose();
    ChassisSpeeds realSpeeds = drive.getFieldRelativeSpeeds();
    Pose bluePose = realPose == null ? null : frame.toBlue(realPose, red);
    Vec2 blueVelocity = realSpeeds == null ? null : frame.velocityToBlue(realSpeeds, red);

    // Xbox: stick up is negative Y, stick left is negative X, right stick right is clockwise.
    double forward = -hid.getLeftY();
    double left = -hid.getLeftX();
    double ccw = -hid.getRightX();
    ChassisCommand driver =
        DriverInput.command(
            forward,
            left,
            ccw,
            pt.safety().stickDeadband(),
            settings.driverMaxSpeed(),
            Constants.DRIVER_MAX_OMEGA);

    ChassisCommand blueOut =
        pt.update(
            new PhantomTrack.Input(
                now,
                mode(),
                bluePose,
                blueVelocity,
                drive.getPoseTimestampSeconds(),
                driver,
                DriverInput.deflection(forward, left, ccw),
                hid.getRightTriggerAxis()));
    ChassisSpeeds realOut = frame.commandToReal(blueOut, red);
    drive.driveFieldRelative(realOut);

    hid.setRumble(GenericHID.RumbleType.kBothRumble, pt.pinWarning() ? 1.0 : 0.0);
    telemetry.publish(now, pt, frame, red, realPose, realOut);
  }

  @Override
  public void end(boolean interrupted) {
    drive.driveFieldRelative(new ChassisSpeeds());
    hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
