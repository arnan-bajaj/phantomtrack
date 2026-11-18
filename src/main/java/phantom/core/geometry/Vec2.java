package phantom.core.geometry;

/**
 * Immutable 2D vector in field coordinates (meters, or m/s for velocities).
 *
 * <p>Field convention matches WPILib: origin at the blue alliance wall corner, +x toward the red
 * alliance wall, +y to the left when standing behind the blue wall, angles counter-clockwise in
 * radians.
 */
public record Vec2(double x, double y) {
  public static final Vec2 ZERO = new Vec2(0.0, 0.0);

  /** Below this length a vector is treated as zero (no defined direction). */
  public static final double EPS = 1e-9;

  public static Vec2 polar(double length, double angleRad) {
    return new Vec2(length * Math.cos(angleRad), length * Math.sin(angleRad));
  }

  public Vec2 plus(Vec2 o) {
    return new Vec2(x + o.x, y + o.y);
  }

  public Vec2 minus(Vec2 o) {
    return new Vec2(x - o.x, y - o.y);
  }

  public Vec2 times(double s) {
    return new Vec2(x * s, y * s);
  }

  public double dot(Vec2 o) {
    return x * o.x + y * o.y;
  }

  /** 2D cross product (z component of the 3D cross product). */
  public double cross(Vec2 o) {
    return x * o.y - y * o.x;
  }

  public double norm() {
    return Math.hypot(x, y);
  }

  public double normSquared() {
    return x * x + y * y;
  }

  public double distanceTo(Vec2 o) {
    return Math.hypot(x - o.x, y - o.y);
  }

  /** Unit vector in the same direction, or ZERO if this vector has (near) zero length. */
  public Vec2 normalized() {
    double n = norm();
    return n < EPS ? ZERO : new Vec2(x / n, y / n);
  }

  /** Same direction, length capped at maxLength. */
  public Vec2 clampNorm(double maxLength) {
    double n = norm();
    if (n <= maxLength || n < EPS) {
      return this;
    }
    return times(maxLength / n);
  }

  /**
   * Angle of this vector in radians, in (-pi, pi]. Returns 0 for the zero vector. (atan2 alone
   * gives -pi for y = -0.0 and x < 0, e.g. (1,0).times(-1); wrap maps that to pi.)
   */
  public double angle() {
    return (Math.abs(x) < EPS && Math.abs(y) < EPS) ? 0.0 : Angles.wrap(Math.atan2(y, x));
  }

  /** Linear interpolation: t=0 gives this, t=1 gives o. */
  public Vec2 lerp(Vec2 o, double t) {
    return new Vec2(x + (o.x - x) * t, y + (o.y - y) * t);
  }

  public boolean isFinite() {
    return Double.isFinite(x) && Double.isFinite(y);
  }

  @Override
  public String toString() {
    return String.format("(%.3f, %.3f)", x, y);
  }
}
