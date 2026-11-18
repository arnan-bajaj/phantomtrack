package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.defense.GhostSession;
import frc.robot.defense.GhostSettings;
import frc.robot.defense.PhantomTrack;
import frc.robot.defense.SafetyConfig;
import frc.robot.drive.DefenseDrive;
import frc.robot.drive.SimDefenseDrive;

/**
 * Wiring and driver controls (Xbox, port 0):
 *
 * <pre>
 *   right trigger (hold)  assist deadman: the assist drives only while it is held past 50%
 *   left stick / right X  drive; ANY movement past the deadband takes control back
 *   D-pad up / right / down / left   Line mirror / Cut-off / Shadow / Zone
 *   right / left bumper   focus: next / previous ghost (auto -> #0 -> #1 -> ... -> auto)
 *   Y                     hold ground on/off
 *   Start                 tell the pose estimator the robot is on the test start mark
 *                         (only with the trigger released; see docs/phase7-test-plan.md)
 *   Back (simulation only) make the reported pose jump 0.75 m, to try the pose-jump rule
 *   rumble                touching a ghost past the pin warning time (1.5 s)
 * </pre>
 */
public final class RobotContainer {
  private final CommandXboxController controller =
      new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final DefenseDrive drive;
  private final PhantomTrack pt =
      new PhantomTrack(SafetyConfig.firstTest(), GhostSettings.firstTest());
  private final AllianceFrame frame = new AllianceFrame(Constants.MIRROR);

  public RobotContainer() {
    SubsystemBase driveSubsystem;
    if (RobotBase.isSimulation()) {
      SimDefenseDrive sim = new SimDefenseDrive(Convert.toWpi(Constants.TEST_START_BLUE));
      drive = sim;
      driveSubsystem = sim;
      controller
          .back()
          .onTrue(Commands.runOnce(() -> sim.injectPoseJump(0.75, 0.0)).ignoringDisable(true));
    } else {
      // >>> Replace these two lines with your drivetrain adapter (robot/README.md). <<<
      // Until then nothing moves: the "drivetrain" is the kinematic sim, so ghosts and telemetry
      // can be checked on the roboRIO with the wheels off.
      SimDefenseDrive placeholder = new SimDefenseDrive(Convert.toWpi(Constants.TEST_START_BLUE));
      DriverStation.reportWarning(
          "PhantomTrack: no real drivetrain wired, using the kinematic sim (nothing will move)",
          false);
      drive = placeholder;
      driveSubsystem = placeholder;
    }

    DefenseDashboard dashboard = new DefenseDashboard(pt.safety(), pt.ghosts());
    driveSubsystem.setDefaultCommand(
        new DefenseTeleop(
            drive, driveSubsystem, controller.getHID(), pt, frame, dashboard, new DefenseTelemetry()));

    controller.povUp().onTrue(select(GhostSession.MODE_LINE_MIRROR));
    controller.povRight().onTrue(select(GhostSession.MODE_CUTOFF));
    controller.povDown().onTrue(select(GhostSession.MODE_SHADOW));
    controller.povLeft().onTrue(select(GhostSession.MODE_ZONE));
    controller.rightBumper().onTrue(Commands.runOnce(() -> pt.cycleFocus(+1)).ignoringDisable(true));
    controller.leftBumper().onTrue(Commands.runOnce(() -> pt.cycleFocus(-1)).ignoringDisable(true));
    controller.y().onTrue(Commands.runOnce(pt::toggleHold).ignoringDisable(true));
    controller.start().onTrue(Commands.runOnce(this::resetToStartMark).ignoringDisable(true));
  }

  private edu.wpi.first.wpilibj2.command.Command select(int mode) {
    return Commands.runOnce(() -> pt.selectMode(mode)).ignoringDisable(true);
  }

  /** Start button: put the pose estimator on the test start mark, never under a held trigger. */
  private void resetToStartMark() {
    if (controller.getHID().getRightTriggerAxis() > pt.safety().triggerThreshold()) {
      DriverStation.reportWarning("PhantomTrack: release the assist trigger before Start", false);
      return;
    }
    Pose2d mark = frame.poseToReal(Constants.TEST_START_BLUE, AllianceFrame.isRedNow());
    if (!drive.resetPose(mark)) {
      DriverStation.reportWarning("PhantomTrack: this drivetrain cannot reset its pose", false);
    }
  }
}
