package frc.robot.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Convert;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.sim.RobotBody;

/**
 * A kinematic swerve for WPILib desktop simulation: the simulator's own {@link RobotBody} (4.5 m/s,
 * 6 m/s^2, stops at the field walls), driven field-relative. No motors, no physics.
 *
 * <p>Behaves like a real drivetrain in the ways the safety code cares about: it stops when the
 * robot is disabled, and it stops if commands stop arriving for {@link #COMMAND_TIMEOUT_SEC}
 * (like WPILib motor safety). {@link #injectPoseJump} adds an error to the reported pose so the
 * pose-jump rule can be tried in sim.
 */
public final class SimDefenseDrive extends SubsystemBase implements DefenseDrive {
  public static final double COMMAND_TIMEOUT_SEC = 0.1;

  // Real field walls only: layout walls (choke, lanes) are virtual, like on a practice carpet.
  private final FieldConfig walls = FieldConfig.placeholder();
  private RobotBody body;
  private Vec2 poseError = Vec2.ZERO;
  private ChassisSpeeds commanded = new ChassisSpeeds();
  private double commandTime = Double.NEGATIVE_INFINITY;
  private double lastPeriodic = Double.NaN;
  private double lastHeading;
  private double omega = 0.0;
  private double poseTime = Timer.getFPGATimestamp();

  public SimDefenseDrive(Pose2d start) {
    body = RobotBody.typical(0, Convert.toCore(start));
    lastHeading = body.heading();
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = Double.isNaN(lastPeriodic) ? 0.0 : Math.max(0.0, Math.min(0.1, now - lastPeriodic));
    lastPeriodic = now;
    boolean live = DriverStation.isEnabled() && now - commandTime <= COMMAND_TIMEOUT_SEC;
    ChassisCommand cmd = live ? Convert.toCore(commanded) : ChassisCommand.STOP;
    if (!cmd.velocity().isFinite() || !Double.isFinite(cmd.omega())) {
      cmd = ChassisCommand.STOP;
    }
    body.step(cmd, dt, walls);
    omega = dt > 0.0 ? Angles.difference(body.heading(), lastHeading) / dt : 0.0;
    lastHeading = body.heading();
    poseTime = now;
  }

  @Override
  public Pose2d getPose() {
    Pose p = body.pose();
    return Convert.toWpi(new Pose(p.position().plus(poseError), p.heading()));
  }

  @Override
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return new ChassisSpeeds(body.velocity().x(), body.velocity().y(), omega);
  }

  @Override
  public void driveFieldRelative(ChassisSpeeds speeds) {
    commanded = speeds;
    commandTime = Timer.getFPGATimestamp();
  }

  @Override
  public double getPoseTimestampSeconds() {
    return poseTime;
  }

  @Override
  public boolean resetPose(Pose2d pose) {
    body = RobotBody.typical(0, Convert.toCore(pose));
    lastHeading = body.heading();
    poseError = Vec2.ZERO;
    return true;
  }

  /** Sim only: make the reported pose wrong by (dx, dy), like a bad vision update. */
  public void injectPoseJump(double dx, double dy) {
    poseError = poseError.plus(new Vec2(dx, dy));
  }
}
