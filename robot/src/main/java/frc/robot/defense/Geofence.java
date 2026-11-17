package frc.robot.defense;

import java.util.List;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Vec2;

/**
 * Keeps assist commands inside a rectangular area. Applies to the ASSIST only: the driver's own
 * commands are never limited.
 *
 * <pre>
 *   area   = [minX, maxX] x [minY, maxY]   (where the robot CENTER may be; blue frame, meters)
 *   fence  = area shrunk by margin on all four sides
 *   d      = distance from the robot center to one wall's fence line, measured inward
 *            (west: x - (minX + margin), east: (maxX - margin) - x, same for y)
 *   v_max(d) = sqrt(2 * brakeDecel * d)   if d > 0     (fastest speed that can still stop in d)
 *            = 0                           if d <= 0    (on or past the fence line)
 *   vx_out = clamp(vx, -v_max(d_west), +v_max(d_east))
 *   vy_out = clamp(vy, -v_max(d_south), +v_max(d_north))
 * </pre>
 *
 * So inside the margin the assist can never command motion toward that wall (only along it or
 * away from it), and it slows down before it gets there. Outside the area entirely the assist is
 * switched off by {@link SafetyGate}; this class would still only allow motion back inward.
 *
 * @param minX west edge of the area (m)
 * @param minY south edge of the area (m)
 * @param maxX east edge of the area (m)
 * @param maxY north edge of the area (m)
 * @param margin how far inside the area the fence line sits (m, for the robot center)
 * @param brakeDecel deceleration assumed when slowing toward the fence (m/s^2)
 */
public record Geofence(
    double minX, double minY, double maxX, double maxY, double margin, double brakeDecel) {

  public Geofence {
    if (!(minX < maxX) || !(minY < maxY)) {
      throw new IllegalArgumentException("area must have minX < maxX and minY < maxY");
    }
    if (!(margin >= 0.0) || !(brakeDecel > 0.0)) {
      throw new IllegalArgumentException("margin must be >= 0 and brakeDecel > 0");
    }
  }

  /**
   * The requested area, cut down to the field so the fence can never be outside the field walls.
   * Returns null if nothing is left (the requested area does not overlap the field).
   */
  public static Geofence inside(
      FieldConfig field,
      double minX,
      double minY,
      double maxX,
      double maxY,
      double margin,
      double brakeDecel) {
    double x0 = Math.max(0.0, minX);
    double y0 = Math.max(0.0, minY);
    double x1 = Math.min(field.length(), maxX);
    double y1 = Math.min(field.width(), maxY);
    if (!(x0 < x1) || !(y0 < y1)) {
      return null;
    }
    return new Geofence(x0, y0, x1, y1, margin, brakeDecel);
  }

  /** True if the robot center is inside the area (edges included). False for NaN. */
  public boolean contains(Vec2 p) {
    return p.x() >= minX && p.x() <= maxX && p.y() >= minY && p.y() <= maxY;
  }

  /** Fastest allowed speed toward a fence line that is d meters away (see class comment). */
  public double maxSpeedToward(double d) {
    return d > 0.0 ? Math.sqrt(2.0 * brakeDecel * d) : 0.0;
  }

  /** The velocity with any part that heads into the fence removed. Never NaN. */
  public Vec2 limit(Vec2 position, Vec2 velocity) {
    if (!position.isFinite() || !velocity.isFinite()) {
      return Vec2.ZERO;
    }
    double west = maxSpeedToward(position.x() - (minX + margin));
    double east = maxSpeedToward((maxX - margin) - position.x());
    double south = maxSpeedToward(position.y() - (minY + margin));
    double north = maxSpeedToward((maxY - margin) - position.y());
    return new Vec2(
        Math.max(-west, Math.min(east, velocity.x())),
        Math.max(-south, Math.min(north, velocity.y())));
  }

  /** Corners of the fence line (area minus margin), counter-clockwise from the south-west. */
  public List<Vec2> fenceCorners() {
    return List.of(
        new Vec2(minX + margin, minY + margin),
        new Vec2(maxX - margin, minY + margin),
        new Vec2(maxX - margin, maxY - margin),
        new Vec2(minX + margin, maxY - margin));
  }
}
