package phantom.sim;

import java.util.List;
import phantom.core.geometry.Vec2;

/**
 * Steering for opponent bots that try to drive around robots in their way (a simple potential
 * field). Only robots ahead of us, within {@code radius}, and closer than the destination count.
 *
 * <pre>
 *   dir   = unit(dest - pos)
 *   perp  = (-dir.y, dir.x)                      (left of travel)
 *   for each obstacle with rel = obs - pos, d = |rel| < radius, 0 < rel.dir < |dest - pos|:
 *     side     = +1 if rel.perp >= 0 (obstacle on our left, or dead ahead) else -1
 *     strength = gain * (1/d - 1/radius)
 *     push    += -side * strength * perp         (steer away from the obstacle's side)
 *   steer = unit(dir + push)
 * </pre>
 */
public final class Avoidance {
  private Avoidance() {}

  /** Unit direction to drive, or ZERO if already at the destination. */
  public static Vec2 steer(Vec2 pos, Vec2 dest, List<Vec2> obstacles, double radius, double gain) {
    Vec2 toDest = dest.minus(pos);
    double distToDest = toDest.norm();
    Vec2 dir = toDest.normalized();
    if (dir.equals(Vec2.ZERO)) {
      return Vec2.ZERO;
    }
    Vec2 perp = new Vec2(-dir.y(), dir.x());
    Vec2 push = Vec2.ZERO;
    for (Vec2 obs : obstacles) {
      Vec2 rel = obs.minus(pos);
      double d = rel.norm();
      double along = rel.dot(dir);
      if (d >= radius || d < 1e-9 || along <= 0 || along >= distToDest) {
        continue;
      }
      double side = rel.dot(perp) >= 0 ? 1.0 : -1.0;
      double strength = gain * (1.0 / d - 1.0 / radius);
      push = push.plus(perp.times(-side * strength));
    }
    Vec2 steer = dir.plus(push).normalized();
    return steer.equals(Vec2.ZERO) ? dir : steer;
  }
}
