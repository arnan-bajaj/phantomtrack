package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.defense.FieldMirror;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/**
 * The alliance boundary. phantom.core always works in the blue frame; on red, every pose coming
 * in and every command going out passes through {@link FieldMirror} (formulas and tests there).
 * The alliance is read once per loop and passed in, so one loop never mixes frames.
 */
public final class AllianceFrame {
  private final FieldMirror mirror;

  public AllianceFrame(FieldMirror mirror) {
    this.mirror = mirror;
  }

  /** Red if the Driver Station says so; blue otherwise (including "not known yet"). */
  public static boolean isRedNow() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }

  public Pose toBlue(Pose2d real, boolean red) {
    Pose p = Convert.toCore(real);
    return red ? mirror.pose(p) : p;
  }

  public Vec2 velocityToBlue(ChassisSpeeds fieldRelative, boolean red) {
    Vec2 v = new Vec2(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond);
    return red ? mirror.velocity(v) : v;
  }

  /**
   * The driver's stick command (from {@link frc.robot.defense.DriverInput}, as if at the blue
   * wall) in the blue frame: unchanged on blue, {@link FieldMirror#driverCommand} on red.
   */
  public ChassisCommand driverToBlue(ChassisCommand stick, boolean red) {
    return red ? mirror.driverCommand(stick) : stick;
  }

  public ChassisSpeeds commandToReal(ChassisCommand blue, boolean red) {
    return Convert.toWpi(red ? mirror.command(blue) : blue);
  }

  public Pose2d poseToReal(Pose blue, boolean red) {
    return Convert.toWpi(red ? mirror.pose(blue) : blue);
  }

  public Translation2d pointToReal(Vec2 blue, boolean red) {
    return Convert.toWpi(red ? mirror.point(blue) : blue);
  }
}
