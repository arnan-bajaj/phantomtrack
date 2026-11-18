package frc.robot;

import frc.robot.defense.FieldMirror;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;

/** Robot-layer constants. Safety numbers live in frc.robot.defense.SafetyConfig. */
public final class Constants {
  private Constants() {}

  public static final int DRIVER_CONTROLLER_PORT = 0;

  /** Driver stick speed at full deflection, first test (m/s). Changeable on the dashboard. */
  public static final double DRIVER_MAX_SPEED = 1.5;

  /** Driver turn rate at full deflection (rad/s). */
  public static final double DRIVER_MAX_OMEGA = Math.PI;

  /**
   * Where the robot is told it is when the driver presses Start (blue frame): middle of the
   * first-test area, facing the opponents' side (-x). On red it is mirrored like everything else.
   * Put a tape mark on the carpet and place the robot there before pressing Start.
   */
  public static final Pose TEST_START_BLUE = new Pose(10.0, 4.035, Math.PI);

  /**
   * How red relates to blue on THIS field. The placeholder field is symmetric both ways, so either
   * works for now. Set it from the 2027 manual's field drawings; FieldMirror has both formulas.
   */
  public static final FieldMirror MIRROR =
      new FieldMirror(
          FieldConfig.placeholder().length(),
          FieldConfig.placeholder().width(),
          FieldMirror.Symmetry.ROTATED);
}
