package phantom.core.field;

import phantom.core.geometry.Vec2;

/**
 * A wall or field element robots cannot drive through: an axis-aligned rectangle.
 *
 * <p>Robots are treated as circles of radius r, so "the robot center may not enter the rectangle
 * grown by r on every side". (Corners of the grown rectangle are square, not rounded: slightly
 * conservative.)
 *
 * @param min lower-left corner (m)
 * @param max upper-right corner (m)
 */
public record Obstacle(Vec2 min, Vec2 max) {
  public Obstacle {
    if (min.x() >= max.x() || min.y() >= max.y()) {
      throw new IllegalArgumentException("min must be below and left of max");
    }
  }

  public static Obstacle of(double x0, double y0, double x1, double y1) {
    return new Obstacle(new Vec2(x0, y0), new Vec2(x1, y1));
  }

  /** True if a robot centered at p (radius r) would overlap this obstacle. */
  public boolean blocks(Vec2 p, double r) {
    return p.x() > min.x() - r && p.x() < max.x() + r && p.y() > min.y() - r && p.y() < max.y() + r;
  }

  /**
   * Moves p out of the grown rectangle along the shortest axis (unchanged if already clear).
   *
   * <pre>
   *   push distances: left = p.x - (min.x - r), right = (max.x + r) - p.x,
   *                   down = p.y - (min.y - r), up    = (max.y + r) - p.y
   *   move by the smallest one in its direction
   * </pre>
   */
  public Vec2 pushOut(Vec2 p, double r) {
    if (!blocks(p, r)) {
      return p;
    }
    double left = p.x() - (min.x() - r);
    double right = (max.x() + r) - p.x();
    double down = p.y() - (min.y() - r);
    double up = (max.y() + r) - p.y();
    double m = Math.min(Math.min(left, right), Math.min(down, up));
    if (m == left) {
      return new Vec2(min.x() - r, p.y());
    } else if (m == right) {
      return new Vec2(max.x() + r, p.y());
    } else if (m == down) {
      return new Vec2(p.x(), min.y() - r);
    }
    return new Vec2(p.x(), max.y() + r);
  }

  /**
   * True if a robot (radius r) driving straight from a to b would hit this obstacle. Slab method
   * on the grown rectangle: the segment a + t (b - a), t in [0, 1], overlaps it in both x and y.
   */
  public boolean blocksPath(Vec2 a, Vec2 b, double r) {
    double t0 = 0.0;
    double t1 = 1.0;
    double[] lo = {min.x() - r, min.y() - r};
    double[] hi = {max.x() + r, max.y() + r};
    double[] start = {a.x(), a.y()};
    double[] d = {b.x() - a.x(), b.y() - a.y()};
    for (int i = 0; i < 2; i++) {
      if (Math.abs(d[i]) < 1e-12) {
        if (start[i] <= lo[i] || start[i] >= hi[i]) {
          return false; // parallel and outside this slab
        }
      } else {
        double ta = (lo[i] - start[i]) / d[i];
        double tb = (hi[i] - start[i]) / d[i];
        t0 = Math.max(t0, Math.min(ta, tb));
        t1 = Math.min(t1, Math.max(ta, tb));
        if (t0 >= t1) {
          return false;
        }
      }
    }
    return true;
  }
}
