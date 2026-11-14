package phantom.core.prediction;

import java.util.ArrayList;
import java.util.List;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Predicts where an opponent is going from game knowledge, not machine learning: FRC robots
 * shuttle between known places (pickups and the goal), and route through gaps when walls are in
 * the way.
 *
 * <pre>
 * Destination (moving, speed >= MOVING_SPEED):
 *   the candidate d in {goal} + pickups maximizing  unit(v) . unit(d - p)   (most "ahead" of them)
 * Destination (slow or stopped):
 *   the goal if they are closer to a pickup than to the goal (just loaded up), else nearest pickup
 *
 * Route from a to d:
 *   [a, d] if the straight line is clear, else [a, g*, d] with g* = argmin |a - g| + |g - d|
 * </pre>
 */
public final class IntentPredictor {
  public static final double MOVING_SPEED = 0.5; // m/s

  private IntentPredictor() {}

  /** Where this opponent is most likely heading right now. */
  public static Vec2 destination(TrackedRobot r, double nowSec, FieldConfig field) {
    Vec2 p = r.positionAt(nowSec);
    Vec2 goal = field.defendedGoal();
    if (r.velocity().norm() < MOVING_SPEED) {
      Vec2 nearestPickup = nearest(p, field.opponentPickups());
      if (nearestPickup == null || p.distanceTo(nearestPickup) < p.distanceTo(goal)) {
        return goal;
      }
      return nearestPickup;
    }
    Vec2 v = r.velocity().normalized();
    Vec2 best = goal;
    double bestScore = v.dot(goal.minus(p).normalized());
    for (Vec2 d : field.opponentPickups()) {
      double score = v.dot(d.minus(p).normalized());
      if (score > bestScore) {
        bestScore = score;
        best = d;
      }
    }
    return best;
  }

  /** Waypoints a robot would drive from a to d, going through the best gate if a wall blocks. */
  public static List<Vec2> route(Vec2 a, Vec2 d, FieldConfig field) {
    List<Vec2> path = new ArrayList<>();
    path.add(a);
    if (!field.gates().isEmpty() && field.pathBlocked(a, d)) {
      Vec2 best = null;
      double bestCost = Double.POSITIVE_INFINITY;
      for (Vec2 g : field.gates()) {
        double c = a.distanceTo(g) + g.distanceTo(d);
        if (c < bestCost) {
          bestCost = c;
          best = g;
        }
      }
      path.add(best);
    }
    path.add(d);
    return path;
  }

  /** Total length of a polyline (m). */
  public static double length(List<Vec2> path) {
    double total = 0.0;
    for (int i = 1; i < path.size(); i++) {
      total += path.get(i - 1).distanceTo(path.get(i));
    }
    return total;
  }

  /** The point at arc length s along the polyline (clamped to its ends). */
  public static Vec2 pointAlong(List<Vec2> path, double s) {
    if (s <= 0) {
      return path.get(0);
    }
    double remaining = s;
    for (int i = 1; i < path.size(); i++) {
      Vec2 a = path.get(i - 1);
      Vec2 b = path.get(i);
      double seg = a.distanceTo(b);
      if (remaining <= seg) {
        return seg < 1e-12 ? b : a.lerp(b, remaining / seg);
      }
      remaining -= seg;
    }
    return path.get(path.size() - 1);
  }

  private static Vec2 nearest(Vec2 p, List<Vec2> points) {
    Vec2 best = null;
    double bestDist = Double.POSITIVE_INFINITY;
    for (Vec2 q : points) {
      double d = q.distanceTo(p);
      if (d < bestDist) {
        bestDist = d;
        best = q;
      }
    }
    return best;
  }
}
