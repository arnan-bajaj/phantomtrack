package phantom.core.rules;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import phantom.core.field.FieldConfig;
import phantom.core.field.Obstacle;
import phantom.core.geometry.Vec2;

/**
 * Picks which way to back off from an opponent we are about to pin: left, right or straight back.
 * Pure geometry, no state.
 *
 * <pre>
 *   away  = unit(p_self - p_threat)                      (straight back)
 *   a     = the opponent's travel direction: unit(v_threat) if it is moving,
 *           otherwise away (it is pushing through us)
 *   LEFT  = (-a.y, a.x),   RIGHT = -LEFT,   BACK = away
 *
 *   for each direction d, over the look-ahead distance D (10 equal steps, 11 points):
 *     hitsWall   = the straight drive p_self -> p_self + D * d crosses an obstacle or leaves the field
 *     clearance  = min over the 11 points of the bumper clearance to walls and obstacles
 *                  (and to other tracked robots: |p - p_robot| - 2 * halfSize)
 *     leavesPath = at the end point, |a x (p_end - p_threat)| >= 2 * halfSize  for every threat
 *                  (our bumpers are out of the lane the opponent is driving along)
 *
 *   bumper clearance at p:
 *     field walls: min(p.x, length - p.x, p.y, width - p.y) - halfSize
 *     obstacle:    max(dx, dy) - halfSize,  dx = max(min.x - p.x, p.x - max.x),  dy likewise
 *                  (square corners, the same convention Obstacle uses)
 *
 *     minAway    = min over threats of d . unit(p_self - p_threat)
 *     intoThreat = minAway < 0                                       (drives toward one)
 *
 *   choose: never a direction that hitsWall or intoThreat (or that the caller found blocked);
 *           then prefer leavesPath; then the largest min(endGap, CLEARANCE_CAP) (see Crowds;
 *           with no other robots it only depends on the threats); then the largest
 *           min(clearance, CLEARANCE_CAP);
 *           then (threat not tracked) BACK; then the end point nearer the defended goal, so we
 *           stay goal-side; then the larger sideways distance from the opponent's path;
 *           then LEFT, RIGHT, BACK.
 *   If no direction qualifies there is no choice: the caller must stop pushing instead. If it
 *   stays wedged, {@link #leastInto} gives the wall-free direction with the largest minAway
 *   (slide out past the threats at the shallowest angle).
 * </pre>
 *
 * With several threats at once, BACK = unit(sum of away_i) and a = BACK (if the sum cancels,
 * a = away from the nearest threat and there is no BACK).
 *
 * <p><b>Crowds</b> ({@link #candidates(FieldConfig, Vec2, List, List, double, Motion)}): every
 * other robot counts too, where it will be over the look-ahead, not only where it is now.
 *
 * <pre>
 *   robots r: every tracked robot plus the threats; "touching" = in contact with us now
 *   our drive along d:  s_k = k D / SAMPLES,   p_k = p_self + s_k d,
 *                       t_k = time to drive s_k from rest at speed v, acceleration a
 *                             ({@link #driveSeconds})
 *   robot r:            u_r = a_r * max(|v_r|, v)   touching and tracked (a pusher keeps coming
 *                                                   along its path a_r, at least as fast as we go)
 *                           = v_r                   tracked, not touching (constant velocity)
 *                           = 0                     not tracked (only a guess where it is)
 *                       q_k = p_r + t_k u_r
 *   gap_k(r) = |p_k - q_k| - 2 * halfSize
 *
 *   intoRobot  = d . unit(p_r - p_self) > 0 for some touching r          (we would push on it)
 *   alongPath  = d . a_r > ALONG_PATH_COS (cos 45 deg) for some touching tracked r
 *                (retreating along a pusher's path: it follows at the same speed, contact goes on)
 *   meetsRobot = gap_k(r) <= CONTACT_TOLERANCE for some k and some r not touching now
 *   endGap     = min over r of gap_SAMPLES(r)                   (how far from everyone we end up)
 *
 *   usable = not hitsWall, intoThreat, intoRobot, alongPath or meetsRobot
 *   chooseExit: only usable candidates not within 30 deg of a blocked direction, ranked
 *           like {@link #choose}.
 * </pre>
 *
 * The caller tries LEFT / RIGHT / BACK first, then the {@link #extraDirections} (both
 * perpendiculars to each touching robot's path and 16 compass directions). If none is usable
 * there is no safe exit and the caller yields: it stops pushing on everyone instead of fleeing.
 */
public final class EscapePlanner {
  private EscapePlanner() {}

  /** Clearance beyond this is "plenty": far walls should not decide between left and right. */
  public static final double CLEARANCE_CAP = 1.0;

  /** The look-ahead is checked at this many equal steps (so SAMPLES + 1 points). */
  public static final int SAMPLES = 10;

  /** Opponent speed below which its travel direction is not trusted (m/s). */
  public static final double MOVING_SPEED = 0.3;

  /** A direction counts as "along a pusher's path" above this d . a (cos 45 deg). */
  public static final double ALONG_PATH_COS = Math.sqrt(0.5);

  /** Number of compass directions tried when LEFT, RIGHT and BACK all fail. */
  public static final int COMPASS = 16;

  /** The three ways out, and OTHER for the extra directions. */
  public enum Option {
    LEFT,
    RIGHT,
    BACK,
    OTHER
  }

  /**
   * Any robot near us, for the crowd checks.
   *
   * @param position where it is (m)
   * @param velocity its velocity (m/s); ZERO if unknown
   * @param tracked true if the camera sees it now
   * @param touching true if it is in contact with us now
   */
  public record Robot(Vec2 position, Vec2 velocity, boolean tracked, boolean touching) {}

  /**
   * How we drive while backing off.
   *
   * @param speed back-off speed (m/s)
   * @param accel acceleration (m/s^2); +infinity = reaches the speed at once
   */
  public record Motion(double speed, double accel) {
    /** The planner's old assumption: 3 m/s at once. */
    public static final Motion INSTANT = new Motion(3.0, Double.POSITIVE_INFINITY);
  }

  /**
   * Something to get away from.
   *
   * @param position where it is (m)
   * @param velocity its velocity (m/s); ZERO if unknown
   * @param tracked true if the camera sees it now; false if the position is a guess (a robot we
   *     lost sight of, or something we only felt)
   */
  public record Threat(Vec2 position, Vec2 velocity, boolean tracked) {}

  /**
   * One evaluated direction.
   *
   * @param option which of the three
   * @param direction unit vector
   * @param hitsWall true if driving the look-ahead distance this way hits a wall or obstacle
   * @param clearance smallest bumper clearance along the look-ahead (m); negative = overlap
   * @param leavesPath true if the end point is out of every threat's path
   * @param lateral sideways distance of the end point from the nearest threat's path (m)
   * @param blind true if any threat's position is a guess (not tracked now)
   * @param intoThreat true if this direction drives toward a threat (d . away < 0)
   * @param goalDistance distance from the end point to the defended goal (m)
   * @param minAway smallest d . unit(p_self - p_threat) over the threats (-1 .. 1)
   * @param intoRobot true if this direction pushes on a robot we are touching
   * @param alongPath true if this direction retreats along a touching robot's path
   * @param meetsRobot true if we would run into a robot we are not touching now
   * @param endGap smallest predicted bumper gap to any robot at the end of the look-ahead (m);
   *     +infinity if there are no robots
   */
  public record Candidate(
      Option option,
      Vec2 direction,
      boolean hitsWall,
      double clearance,
      boolean leavesPath,
      double lateral,
      boolean blind,
      boolean intoThreat,
      double goalDistance,
      double minAway,
      boolean intoRobot,
      boolean alongPath,
      boolean meetsRobot,
      double endGap) {

    /** True if this is a way out: no wall, no pushing on anyone, no running into anyone. */
    public boolean usable() {
      return !hitsWall && !intoThreat && !intoRobot && !alongPath && !meetsRobot;
    }
  }

  /** True if driving along dir moves toward any threat: d . unit(p_self - p_threat) < 0. */
  public static boolean drivesInto(Vec2 self, List<Threat> threats, Vec2 dir) {
    return minAway(self, threats, dir) < -1e-9;
  }

  /** Smallest d . unit(p_self - p_threat) over the threats (1 if there are none). */
  public static double minAway(Vec2 self, List<Threat> threats, Vec2 dir) {
    double m = 1.0;
    for (Threat t : threats) {
      m = Math.min(m, dir.dot(away(self, t.position())));
    }
    return m;
  }

  /**
   * Last resort when {@link #choose} finds nothing: the candidate that does not hit a wall and is
   * not blocked with the largest minAway (ties: list order). Empty if every one hits a wall.
   */
  public static Optional<Candidate> leastInto(List<Candidate> candidates, Set<Option> blocked) {
    Candidate best = null;
    for (Candidate c : candidates) {
      if (c.hitsWall() || blocked.contains(c.option())) {
        continue;
      }
      if (best == null || c.minAway() > best.minAway() + 1e-9) {
        best = c;
      }
    }
    return Optional.ofNullable(best);
  }

  /** Bumper clearance of a robot centered at p to the field walls and obstacles (m). */
  public static double clearance(FieldConfig field, Vec2 p) {
    double r = field.robotHalfSize();
    double best =
        Math.min(Math.min(p.x(), field.length() - p.x()), Math.min(p.y(), field.width() - p.y())) - r;
    for (Obstacle o : field.obstacles()) {
      double dx = Math.max(o.min().x() - p.x(), p.x() - o.max().x());
      double dy = Math.max(o.min().y() - p.y(), p.y() - o.max().y());
      best = Math.min(best, Math.max(dx, dy) - r);
    }
    return best;
  }

  /**
   * Smallest clearance at the SAMPLES + 1 points from {@code from} to {@code from + distance * dir}.
   *
   * @param otherRobots positions of robots to keep away from (their bumper clearance counts too)
   */
  public static double pathClearance(
      FieldConfig field, Vec2 from, Vec2 dir, double distance, List<Vec2> otherRobots) {
    double best = Double.POSITIVE_INFINITY;
    for (int k = 0; k <= SAMPLES; k++) {
      Vec2 p = from.plus(dir.times(distance * k / SAMPLES));
      best = Math.min(best, clearance(field, p));
      for (Vec2 robot : otherRobots) {
        best = Math.min(best, p.distanceTo(robot) - 2.0 * field.robotHalfSize());
      }
    }
    return best;
  }

  /** True if driving straight from {@code from} for {@code distance} along dir hits a wall. */
  public static boolean hitsWall(FieldConfig field, Vec2 from, Vec2 dir, double distance) {
    Vec2 end = from.plus(dir.times(distance));
    double r = field.robotHalfSize();
    double eps = 1e-9;
    if (end.x() < r - eps
        || end.x() > field.length() - r + eps
        || end.y() < r - eps
        || end.y() > field.width() - r + eps) {
      return true;
    }
    return field.pathBlocked(from, end);
  }

  /** The opponent's travel direction as seen from {@code self} (see the class comment). */
  public static Vec2 pathDirection(Vec2 self, Threat threat) {
    if (threat.velocity().norm() > MOVING_SPEED) {
      return threat.velocity().normalized();
    }
    return away(self, threat.position());
  }

  /**
   * Time to drive s meters from rest at up to {@code speed}, accelerating at {@code accel}:
   *
   * <pre>
   *   d_acc = v^2 / (2 a)
   *   T     = sqrt(2 s / a)               if s <= d_acc
   *         = v / a + (s - d_acc) / v     otherwise        (a = +infinity: T = s / v)
   * </pre>
   */
  public static double driveSeconds(double s, double speed, double accel) {
    if (s <= 0.0) {
      return 0.0;
    }
    if (accel == Double.POSITIVE_INFINITY) {
      return s / speed;
    }
    double dAcc = speed * speed / (2.0 * accel);
    return s <= dAcc ? Math.sqrt(2.0 * s / accel) : speed / accel + (s - dAcc) / speed;
  }

  /** Where robot r is predicted to be t seconds from now (see the class comment). */
  public static Vec2 predict(Vec2 self, Robot r, double t, double ourSpeed) {
    if (!r.tracked()) {
      return r.position();
    }
    Vec2 u;
    if (r.touching()) {
      Vec2 a = pathDirection(self, new Threat(r.position(), r.velocity(), true));
      u = a.times(Math.max(r.velocity().norm(), ourSpeed));
    } else {
      u = r.velocity();
    }
    return r.position().plus(u.times(t));
  }

  /**
   * The 16 compass directions (0, 22.5, ... deg) and both perpendiculars to the path of each
   * touching robot, all as Option.OTHER. Tried when LEFT, RIGHT and BACK all fail.
   */
  public static List<Vec2> extraDirections(Vec2 self, List<Robot> robots) {
    List<Vec2> out = new ArrayList<>();
    for (Robot r : robots) {
      if (r.touching() && r.tracked()) {
        Vec2 a = pathDirection(self, new Threat(r.position(), r.velocity(), true));
        out.add(new Vec2(-a.y(), a.x()));
        out.add(new Vec2(a.y(), -a.x()));
      }
    }
    for (int i = 0; i < COMPASS; i++) {
      out.add(Vec2.polar(1.0, 2.0 * Math.PI * i / COMPASS));
    }
    return out;
  }

  private static Vec2 away(Vec2 self, Vec2 threat) {
    Vec2 away = self.minus(threat).normalized();
    return away.equals(Vec2.ZERO) ? new Vec2(1.0, 0.0) : away;
  }

  /**
   * Evaluates LEFT, RIGHT and BACK.
   *
   * @param threats what we are backing away from (at least one)
   * @param distance look-ahead distance (m)
   * @param otherRobots other robots to keep clear of (not the threats)
   */
  public static List<Candidate> candidates(
      FieldConfig field, Vec2 self, List<Threat> threats, double distance, List<Vec2> otherRobots) {
    List<Robot> robots = new ArrayList<>();
    for (Vec2 p : otherRobots) {
      robots.add(new Robot(p, Vec2.ZERO, true, false));
    }
    return candidates(field, self, threats, robots, distance, Motion.INSTANT);
  }

  /**
   * Evaluates LEFT, RIGHT and BACK with the crowd checks (see the class comment).
   *
   * @param robots every other robot we know of (not the threats)
   * @param motion how we drive while backing off
   */
  public static List<Candidate> candidates(
      FieldConfig field,
      Vec2 self,
      List<Threat> threats,
      List<Robot> robots,
      double distance,
      Motion motion) {
    if (threats.isEmpty()) {
      throw new IllegalArgumentException("need at least one threat");
    }
    Threat nearest = threats.get(0);
    Vec2 sum = Vec2.ZERO;
    for (Threat t : threats) {
      sum = sum.plus(away(self, t.position()));
      if (t.position().distanceTo(self) < nearest.position().distanceTo(self)) {
        nearest = t;
      }
    }
    Vec2 back = threats.size() == 1 ? away(self, nearest.position()) : sum.normalized();
    Vec2 axis;
    if (threats.size() == 1) {
      axis = pathDirection(self, nearest);
    } else {
      axis = back.equals(Vec2.ZERO) ? away(self, nearest.position()) : back;
    }
    Vec2 left = new Vec2(-axis.y(), axis.x());

    List<Robot> all = withThreats(threats, robots);
    List<Candidate> out = new ArrayList<>();
    out.add(evaluate(field, self, threats, nearest, Option.LEFT, left, distance, all, motion));
    out.add(evaluate(
        field, self, threats, nearest, Option.RIGHT, left.times(-1.0), distance, all, motion));
    if (!back.equals(Vec2.ZERO)) {
      out.add(evaluate(field, self, threats, nearest, Option.BACK, back, distance, all, motion));
    }
    return out;
  }

  /** The {@link #extraDirections}, evaluated like {@link #candidates} (all Option.OTHER). */
  public static List<Candidate> extraCandidates(
      FieldConfig field,
      Vec2 self,
      List<Threat> threats,
      List<Robot> robots,
      double distance,
      Motion motion) {
    Threat nearest = nearest(self, threats);
    List<Robot> all = withThreats(threats, robots);
    List<Candidate> out = new ArrayList<>();
    for (Vec2 d : extraDirections(self, all)) {
      out.add(evaluate(field, self, threats, nearest, Option.OTHER, d, distance, all, motion));
    }
    return out;
  }

  /** One direction, evaluated like {@link #candidates} (Option.OTHER). */
  public static Candidate evaluate(
      FieldConfig field,
      Vec2 self,
      List<Threat> threats,
      List<Robot> robots,
      Vec2 dir,
      double distance,
      Motion motion) {
    return evaluate(field, self, threats, nearest(self, threats), Option.OTHER, dir, distance,
        withThreats(threats, robots), motion);
  }

  private static Threat nearest(Vec2 self, List<Threat> threats) {
    Threat nearest = threats.get(0);
    for (Threat t : threats) {
      if (t.position().distanceTo(self) < nearest.position().distanceTo(self)) {
        nearest = t;
      }
    }
    return nearest;
  }

  /** The threats are touching robots too. */
  private static List<Robot> withThreats(List<Threat> threats, List<Robot> robots) {
    List<Robot> all = new ArrayList<>(robots);
    for (Threat t : threats) {
      all.add(new Robot(t.position(), t.velocity(), t.tracked(), true));
    }
    return all;
  }

  private static Candidate evaluate(
      FieldConfig field,
      Vec2 self,
      List<Threat> threats,
      Threat nearest,
      Option option,
      Vec2 dir,
      double distance,
      List<Robot> robots,
      Motion motion) {
    Vec2 end = self.plus(dir.times(distance));
    boolean leaves = true;
    for (Threat t : threats) {
      double lateral = Math.abs(pathDirection(self, t).cross(end.minus(t.position())));
      leaves &= t.tracked() && lateral >= 2.0 * field.robotHalfSize();
    }
    double lateral = Math.abs(pathDirection(self, nearest).cross(end.minus(nearest.position())));
    boolean blind = false;
    for (Threat t : threats) {
      blind |= !t.tracked();
    }

    // Static clearance (walls, obstacles; other robots where they are now).
    List<Vec2> others = new ArrayList<>();
    for (Robot r : robots) {
      if (!r.touching()) {
        others.add(r.position());
      }
    }
    double clearance = pathClearance(field, self, dir, distance, others);
    boolean wall = hitsWall(field, self, dir, distance);

    // Robots: pushing on them, retreating along their path, running into them (see class comment).
    boolean intoRobot = false;
    boolean alongPath = false;
    boolean meets = false;
    double endGap = Double.POSITIVE_INFINITY;
    double v = motion.speed();
    double reach = 2.0 * field.robotHalfSize();
    for (Robot r : robots) {
      if (r.touching()) {
        intoRobot |= dir.dot(r.position().minus(self).normalized()) > 1e-9;
        if (r.tracked()) {
          Vec2 a = pathDirection(self, new Threat(r.position(), r.velocity(), true));
          alongPath |= dir.dot(a) > ALONG_PATH_COS + 1e-12;
        }
      }
      for (int k = 0; k <= SAMPLES; k++) {
        double s = distance * k / SAMPLES;
        double t = driveSeconds(s, v, motion.accel());
        double gap = self.plus(dir.times(s)).distanceTo(predict(self, r, t, v)) - reach;
        if (!r.touching() && gap <= PinTimer.CONTACT_TOLERANCE) {
          meets = true;
        }
        if (k == SAMPLES) {
          endGap = Math.min(endGap, gap);
        }
      }
    }
    return new Candidate(
        option,
        dir,
        wall,
        clearance,
        leaves,
        lateral,
        blind,
        drivesInto(self, threats, dir),
        end.distanceTo(field.defendedGoal()),
        minAway(self, threats, dir),
        intoRobot,
        alongPath,
        meets,
        endGap);
  }

  /**
   * The best candidate that does not hit a wall, does not drive toward a threat and is not in
   * {@code blocked}; empty if there is
   * none (then the caller must stop pushing and stay put). Order of preference is in the class
   * comment.
   */
  public static Optional<Candidate> choose(List<Candidate> candidates, Set<Option> blocked) {
    Candidate best = null;
    for (Candidate c : candidates) {
      if (c.hitsWall() || c.intoThreat() || blocked.contains(c.option())) {
        continue;
      }
      if (best == null || better(c, best)) {
        best = c;
      }
    }
    return Optional.ofNullable(best);
  }

  /**
   * The best {@link Candidate#usable} candidate whose direction is not within 30 deg of a
   * {@code blocked} direction (d . b > cos 30 deg); empty if there is none (then the caller
   * yields). Order of preference is in the class comment.
   */
  public static Optional<Candidate> chooseExit(List<Candidate> candidates, List<Vec2> blocked) {
    Candidate best = null;
    for (Candidate c : candidates) {
      if (!c.usable() || isBlocked(c.direction(), blocked)) {
        continue;
      }
      if (best == null || better(c, best)) {
        best = c;
      }
    }
    return Optional.ofNullable(best);
  }

  /** True if d is within 30 deg of one of the blocked directions. */
  public static boolean isBlocked(Vec2 d, List<Vec2> blocked) {
    for (Vec2 b : blocked) {
      if (d.dot(b) > Math.sqrt(0.75)) {
        return true;
      }
    }
    return false;
  }

  private static boolean better(Candidate c, Candidate best) {
    if (c.leavesPath() != best.leavesPath()) {
      return c.leavesPath();
    }
    double cg = Math.min(c.endGap(), CLEARANCE_CAP);
    double bg = Math.min(best.endGap(), CLEARANCE_CAP);
    if (Math.abs(cg - bg) > 1e-9) {
      return cg > bg;
    }
    double cc = Math.min(c.clearance(), CLEARANCE_CAP);
    double bc = Math.min(best.clearance(), CLEARANCE_CAP);
    if (Math.abs(cc - bc) > 1e-9) {
      return cc > bc;
    }
    if (c.blind() && (c.option() == Option.BACK) != (best.option() == Option.BACK)) {
      return c.option() == Option.BACK; // cannot see it: straight back cannot rub along it
    }
    if (Math.abs(c.goalDistance() - best.goalDistance()) > 1e-6) {
      return c.goalDistance() < best.goalDistance();
    }
    if (Math.abs(c.lateral() - best.lateral()) > 1e-6) {
      return c.lateral() > best.lateral();
    }
    return c.option().ordinal() < best.option().ordinal();
  }
}
