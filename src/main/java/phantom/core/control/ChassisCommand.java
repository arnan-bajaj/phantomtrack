package phantom.core.control;

import phantom.core.geometry.Vec2;

/**
 * Field-relative drive command.
 *
 * @param velocity field-relative translation velocity (m/s)
 * @param omega rotation rate (rad/s, CCW positive)
 */
public record ChassisCommand(Vec2 velocity, double omega) {
  public static final ChassisCommand STOP = new ChassisCommand(Vec2.ZERO, 0.0);
}
