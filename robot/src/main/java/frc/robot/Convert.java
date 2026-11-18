package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/** WPILib geometry types to phantom.core types and back. Same units and frame on both sides. */
public final class Convert {
  private Convert() {}

  public static Pose toCore(Pose2d p) {
    return new Pose(p.getX(), p.getY(), p.getRotation().getRadians());
  }

  public static Pose2d toWpi(Pose p) {
    return new Pose2d(p.position().x(), p.position().y(), new Rotation2d(p.heading()));
  }

  public static Translation2d toWpi(Vec2 v) {
    return new Translation2d(v.x(), v.y());
  }

  /** Field-relative speeds as a core command (velocity + omega). */
  public static ChassisCommand toCore(ChassisSpeeds s) {
    return new ChassisCommand(
        new Vec2(s.vxMetersPerSecond, s.vyMetersPerSecond), s.omegaRadiansPerSecond);
  }

  public static ChassisSpeeds toWpi(ChassisCommand c) {
    return new ChassisSpeeds(c.velocity().x(), c.velocity().y(), c.omega());
  }
}
