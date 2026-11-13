package phantom.core.geometry;

import java.util.OptionalDouble;

/**
 * Earliest time we can reach a target moving at constant velocity.
 *
 * <p>With r = p_target - p_self, target velocity v and our top speed s, we need |r + v t| = s t:
 *
 * <pre>
 *   (v.v - s^2) t^2 + 2 (r.v) t + r.r = 0
 * </pre>
 *
 * The answer is the smallest positive root. When v.v = s^2 the equation is linear: 2 (r.v) t + r.r
 * = 0.
 */
public final class Intercept {
  private static final double EPS = 1e-9;

  private Intercept() {}

  /**
   * @param relativePosition r = target position - our position (m)
   * @param targetVelocity v = target velocity (m/s)
   * @param ourSpeed s = our top speed (m/s), must be > 0
   * @return earliest intercept time in seconds, or empty if we can never catch it
   */
  public static OptionalDouble time(Vec2 relativePosition, Vec2 targetVelocity, double ourSpeed) {
    if (ourSpeed <= 0) {
      throw new IllegalArgumentException("ourSpeed must be > 0");
    }
    double c = relativePosition.normSquared();
    if (c < EPS) {
      return OptionalDouble.of(0.0); // already there
    }
    double a = targetVelocity.normSquared() - ourSpeed * ourSpeed;
    double b = 2.0 * relativePosition.dot(targetVelocity);

    if (Math.abs(a) < EPS) {
      // Same speed: linear. b*t + c = 0 -> t = -c / b, positive only if b < 0 (closing).
      if (b >= -EPS) {
        return OptionalDouble.empty();
      }
      return OptionalDouble.of(-c / b);
    }

    double disc = b * b - 4.0 * a * c;
    if (disc < 0) {
      return OptionalDouble.empty();
    }
    double sq = Math.sqrt(disc);
    double t1 = (-b - sq) / (2.0 * a);
    double t2 = (-b + sq) / (2.0 * a);
    double best = Double.POSITIVE_INFINITY;
    if (t1 > EPS) {
      best = t1;
    }
    if (t2 > EPS && t2 < best) {
      best = t2;
    }
    return Double.isInfinite(best) ? OptionalDouble.empty() : OptionalDouble.of(best);
  }
}
