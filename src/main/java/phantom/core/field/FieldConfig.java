package phantom.core.field;

import java.util.List;
import phantom.core.geometry.Vec2;

/**
 * Field geometry the defense code needs. Kept as plain data so the 2027 field can be dropped in at
 * kickoff without touching any algorithm.
 *
 * @param length field length along x (m)
 * @param width field width along y (m)
 * @param defendedGoal the point opponents score at, which we defend (m)
 * @param opponentPickups where opponents collect game pieces (m)
 * @param robotHalfSize half of the bumper-to-bumper width of a robot (m)
 */
public record FieldConfig(
    double length,
    double width,
    Vec2 defendedGoal,
    List<Vec2> opponentPickups,
    double robotHalfSize) {

  /**
   * Placeholder geometry: a generic FRC-sized field. Goal and pickup locations are made up for
   * testing. Replace with real 2027 geometry after kickoff.
   */
  public static FieldConfig placeholder() {
    return new FieldConfig(
        16.54,
        8.07,
        new Vec2(13.0, 4.035),
        List.of(new Vec2(1.5, 1.2), new Vec2(1.5, 6.9), new Vec2(3.0, 4.035)),
        0.45);
  }

  /** Clamp a point so a robot centered there stays fully inside the field walls. */
  public Vec2 clampInside(Vec2 p) {
    double m = robotHalfSize;
    return new Vec2(Math.max(m, Math.min(length - m, p.x())), Math.max(m, Math.min(width - m, p.y())));
  }
}
