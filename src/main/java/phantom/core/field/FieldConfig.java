package phantom.core.field;

import java.util.List;
import phantom.core.geometry.Vec2;

/**
 * Field geometry the defense code needs. Kept as plain data so the 2027 field can be dropped in at
 * kickoff without touching any algorithm.
 *
 * @param name short label for reports
 * @param length field length along x (m)
 * @param width field width along y (m)
 * @param defendedGoal the point opponents score at, which we defend (m)
 * @param opponentPickups where opponents collect game pieces (m)
 * @param robotHalfSize half of the bumper-to-bumper width of a robot (m)
 * @param obstacles walls and field elements nobody can drive through
 * @param gates centers of the gaps between obstacles that robots route through
 */
public record FieldConfig(
    String name,
    double length,
    double width,
    Vec2 defendedGoal,
    List<Vec2> opponentPickups,
    double robotHalfSize,
    List<Obstacle> obstacles,
    List<Vec2> gates) {

  public FieldConfig {
    opponentPickups = List.copyOf(opponentPickups);
    obstacles = List.copyOf(obstacles);
    gates = List.copyOf(gates);
  }

  /** Open field: no obstacles. */
  public FieldConfig(
      double length,
      double width,
      Vec2 defendedGoal,
      List<Vec2> opponentPickups,
      double robotHalfSize) {
    this("Open", length, width, defendedGoal, opponentPickups, robotHalfSize, List.of(), List.of());
  }

  private static final double LENGTH = 16.54;
  private static final double WIDTH = 8.07;
  private static final Vec2 GOAL = new Vec2(13.0, 4.035);
  private static final List<Vec2> PICKUPS =
      List.of(new Vec2(1.5, 1.2), new Vec2(1.5, 6.9), new Vec2(3.0, 4.035));
  private static final double HALF = 0.45;
  /** Walls in the made-up layouts sit 4 m in front of the goal, 0.4 m thick. */
  private static final double WALL_X0 = 8.6;
  private static final double WALL_X1 = 9.0;

  /**
   * Placeholder geometry: a generic FRC-sized open field. Goal and pickup locations are made up for
   * testing. Replace with real 2027 geometry after kickoff.
   */
  public static FieldConfig placeholder() {
    return new FieldConfig(LENGTH, WIDTH, GOAL, PICKUPS, HALF);
  }

  /** Made-up layout: a wall across the field 4 m in front of the goal with one 2.47 m gap. */
  public static FieldConfig choke() {
    return new FieldConfig(
        "Choke",
        LENGTH,
        WIDTH,
        GOAL,
        PICKUPS,
        HALF,
        List.of(Obstacle.of(WALL_X0, 0.0, WALL_X1, 2.8), Obstacle.of(WALL_X0, 5.27, WALL_X1, WIDTH)),
        List.of(new Vec2((WALL_X0 + WALL_X1) / 2.0, 4.035)));
  }

  /** Made-up layout: the same wall with two 2.0 m lanes, so opponents can pick the open one. */
  public static FieldConfig twoLanes() {
    return new FieldConfig(
        "Two lanes",
        LENGTH,
        WIDTH,
        GOAL,
        PICKUPS,
        HALF,
        List.of(
            Obstacle.of(WALL_X0, 0.0, WALL_X1, 1.0),
            Obstacle.of(WALL_X0, 3.0, WALL_X1, 5.07),
            Obstacle.of(WALL_X0, 7.07, WALL_X1, WIDTH)),
        List.of(new Vec2((WALL_X0 + WALL_X1) / 2.0, 2.0), new Vec2((WALL_X0 + WALL_X1) / 2.0, 6.07)));
  }

  /**
   * Nearest legal spot for a robot centered at p: inside the field walls and out of every
   * obstacle. An obstacle pushes p out along the shortest of its four axis exits (left, right,
   * down, up; ties in that order) that is inside the field walls:
   *
   * <pre>
   *   exits: (min.x - r, p.y), (max.x + r, p.y), (p.x, min.y - r), (p.x, max.y + r)
   *   keep those with r <= x <= length - r and r <= y <= width - r, take the nearest
   * </pre>
   *
   * (An exit through a field wall would be clamped straight back into the obstacle when the
   * obstacle touches that wall.) If no exit is on the field, it falls back to the plain shortest
   * push, clamped to the walls.
   */
  public Vec2 clampInside(Vec2 p) {
    Vec2 q = clampToWalls(p);
    for (int pass = 0; pass < 2; pass++) { // twice, in case one push lands in a neighbor
      for (Obstacle o : obstacles) {
        if (o.blocks(q, robotHalfSize)) {
          q = exitOnTheField(o, q);
        }
      }
    }
    return q;
  }

  private Vec2 exitOnTheField(Obstacle o, Vec2 q) {
    double r = robotHalfSize;
    Vec2[] exits = {
      new Vec2(o.min().x() - r, q.y()),
      new Vec2(o.max().x() + r, q.y()),
      new Vec2(q.x(), o.min().y() - r),
      new Vec2(q.x(), o.max().y() + r)
    };
    Vec2 best = null;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (Vec2 e : exits) {
      double d = e.distanceTo(q);
      if (insideWalls(e) && d < bestDistance) {
        best = e;
        bestDistance = d;
      }
    }
    return best != null ? best : clampToWalls(o.pushOut(q, r));
  }

  private boolean insideWalls(Vec2 p) {
    double m = robotHalfSize;
    double eps = 1e-9;
    return p.x() >= m - eps && p.x() <= length - m + eps && p.y() >= m - eps
        && p.y() <= width - m + eps;
  }

  private Vec2 clampToWalls(Vec2 p) {
    double m = robotHalfSize;
    return new Vec2(Math.max(m, Math.min(length - m, p.x())), Math.max(m, Math.min(width - m, p.y())));
  }

  /** True if a robot driving straight from a to b would hit an obstacle. */
  public boolean pathBlocked(Vec2 a, Vec2 b) {
    for (Obstacle o : obstacles) {
      if (o.blocksPath(a, b, robotHalfSize)) {
        return true;
      }
    }
    return false;
  }
}
