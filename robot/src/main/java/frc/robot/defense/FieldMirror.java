package frc.robot.defense;

import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/**
 * Converts between the real field frame and the blue frame that phantom.core always works in.
 *
 * <p>When we are blue, the two frames are the same. When we are red, the field looks to us the way
 * it looks to blue after applying the field's symmetry, so we apply that symmetry to every pose on
 * the way in and to every command on the way out:
 *
 * <pre>
 *   ROTATED  (field turned 180 degrees about its center, e.g. 2022, 2025)
 *     x' = L - x     y' = W - y     heading' = wrap(heading + pi)
 *     vx' = -vx      vy' = -vy      omega' = omega
 *
 *   MIRRORED (field reflected across the center line x = L / 2, e.g. 2023, 2024)
 *     x' = L - x     y' = y         heading' = wrap(pi - heading)
 *     vx' = -vx      vy' = vy       omega' = -omega
 * </pre>
 *
 * Both maps undo themselves (apply twice = no change), so the same method converts in both
 * directions. The DRIVER's stick is different: see {@link #driverCommand}. L and W must be the size of the field the drivetrain's pose estimator uses. Which
 * symmetry applies changes from year to year: check the game manual's field drawings.
 *
 * @param fieldLength L, along x (m)
 * @param fieldWidth W, along y (m)
 * @param symmetry how the red half relates to the blue half
 */
public record FieldMirror(double fieldLength, double fieldWidth, Symmetry symmetry) {
  public enum Symmetry {
    ROTATED,
    MIRRORED
  }

  public Vec2 point(Vec2 p) {
    return symmetry == Symmetry.ROTATED
        ? new Vec2(fieldLength - p.x(), fieldWidth - p.y())
        : new Vec2(fieldLength - p.x(), p.y());
  }

  public double heading(double headingRad) {
    return symmetry == Symmetry.ROTATED
        ? Angles.wrap(headingRad + Math.PI)
        : Angles.wrap(Math.PI - headingRad);
  }

  public Pose pose(Pose p) {
    return new Pose(point(p.position()), heading(p.heading()));
  }

  public Vec2 velocity(Vec2 v) {
    return symmetry == Symmetry.ROTATED ? new Vec2(-v.x(), -v.y()) : new Vec2(-v.x(), v.y());
  }

  public double omega(double omegaRadPerSec) {
    return symmetry == Symmetry.ROTATED ? omegaRadPerSec : -omegaRadPerSec;
  }

  public ChassisCommand command(ChassisCommand c) {
    return new ChassisCommand(velocity(c.velocity()), omega(c.omega()));
  }

  /**
   * A RED driver's stick command in the blue frame. {@link DriverInput} builds the command as if
   * the driver stood at the blue wall (stick forward = +x). A red driver stands at the red wall,
   * turned 180 degrees: a person is rotated, never reflected, whatever the field's symmetry. So
   * their command on the real field is the stick command rotated 180 degrees, and the blue frame
   * sees that through the field's symmetry:
   *
   * <pre>
   *   real  = (-vx, -vy, omega)                       (rotated 180 degrees)
   *   blue  = command(real)
   *     ROTATED:  (vx, vy, omega)                      (the same as the stick)
   *     MIRRORED: (vx, -vy, -omega)
   * </pre>
   *
   * Using the stick as a blue-frame command directly is right for ROTATED only; on a MIRRORED
   * field it reverses the red driver's left/right and turning.
   */
  public ChassisCommand driverCommand(ChassisCommand stick) {
    ChassisCommand real =
        new ChassisCommand(new Vec2(-stick.velocity().x(), -stick.velocity().y()), stick.omega());
    return command(real);
  }
}
