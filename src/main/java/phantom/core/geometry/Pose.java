package phantom.core.geometry;

/** Robot pose on the field: position (m) and heading (rad, CCW from +x). */
public record Pose(Vec2 position, double heading) {
  public Pose(double x, double y, double heading) {
    this(new Vec2(x, y), heading);
  }

  @Override
  public String toString() {
    return String.format("Pose[%s, %.1f deg]", position, Math.toDegrees(heading));
  }
}
