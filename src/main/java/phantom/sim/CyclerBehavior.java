package phantom.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Vec2;

/**
 * Drives pickup -> goal -> pickup -> ... forever and counts a cycle each time it reaches scoring
 * range of the goal.
 *
 * <ul>
 *   <li><b>Cycler</b> ignores defenders and drives straight through them.
 *   <li><b>Evader</b> steers around robots in its path ({@link Avoidance}), reacting to where they
 *       were {@code reactionSec} ago (rounded back to the 20 ms sim step), like a human driver
 *       (default 0.25 s). If it is stuck (under
 *       0.3 m/s for 1 s) it gives up dodging and pushes straight for 1 s.
 * </ul>
 *
 * <p>With obstacles on the field, both route through a gate when the straight line is blocked.
 * Gate cost = |p - gate| + |gate - dest| (+ 1.5 m to switch away from the gate already chosen).
 * An Evader also adds 4 m for every robot it sees (with its reaction delay) within 2 m of a gate,
 * so it picks the open lane. Any bot stuck at a gate for 1 s (e.g. head-on with another robot)
 * treats that gate as blocked for 3 s (+10 m) and tries another, like a driver backing out.
 *
 * <p><b>Head-on in a gap (Cyclers only, audit M5).</b> Two Cyclers that meet head-on inside a gap
 * would push each other forever (equal effort stalls, and inside the gap the straight path is not
 * blocked, so the gate rule above never runs). So: when another Cycler with a higher id is
 * touching us (bumper gap at most 0.05 m), both drive into each other (pushing effort above 0.5,
 * as in {@link Contact#effort}) and the midpoint of the two is within 1.0 m of a gate, for 1.0 s
 * in a row, and over that second neither moved 0.3 m or more (net displacement, 0.3 m/s x 1 s; not
 * the body's velocity, which collisions keep high in a crowded jam), the lower id backs out to
 *
 * <pre>
 *   u    = unit(gate - p_me)          (or unit(gate - dest_other) if we sit on the gate)
 *   perp = (-u.y, u.x)
 *   side = -perp if perp . (dest_other - gate) > 0, else perp   (away from where the other goes)
 *   Y    = clampInside(gate - 1.5 u + 1.2 side)
 * </pre>
 *
 * until the other has passed ({@code (p_other - p_me) . u < 0}: it is behind us, seen from the
 * gate), or for at most 5 s. A Cycler never backs out from our robot or from a partner: driving
 * through defenders is what it models. Evaders already have their own way out (bulldoze, gate
 * switching) and are not changed.
 */
public final class CyclerBehavior implements OpponentBehavior {
  private final Vec2 pickup;
  private final Vec2 goal;
  private final double pickupRadius;
  private final double scoreRadius;
  private final double dwellSeconds;
  private final boolean evasive;
  private final double reactionSec;

  /** Typical human reaction time to something moving in front of you (s). */
  public static final double HUMAN_REACTION_SEC = 0.25;

  // Evader tuning
  private static final double AVOID_RADIUS = 3.0;
  private static final double AVOID_GAIN = 1.5;
  private static final double STUCK_SPEED = 0.3;
  private static final double STUCK_SECONDS = 1.0;
  private static final double BULLDOZE_SECONDS = 1.0;
  private static final double SWITCH_PENALTY = 1.5;
  private static final double CROWDED_RADIUS = 2.0;
  private static final double CROWDED_PENALTY = 4.0;
  private static final double BLOCKED_PENALTY = 10.0;
  private static final double BLOCKED_SECONDS = 3.0;
  // Cycler head-on yield (audit M5)
  static final double GAP_RADIUS = 1.0;
  static final double HEAD_ON_SECONDS = 1.0;
  static final double HEAD_ON_EFFORT = 0.5;
  static final double TOUCH_GAP = 0.05;
  static final double YIELD_BACK = 1.5;
  static final double YIELD_SIDE = 1.2;
  static final double YIELD_MAX_SECONDS = 5.0;
  private int committedGate = -1;
  private int blockedGate = -1;
  private double blockedUntil = Double.NEGATIVE_INFINITY;
  private double gateStuckFor = 0.0;
  private double stuckFor = 0.0;
  private double bulldozeUntil = -1.0;
  private int headOnWith = -1;
  private Vec2 headOnAnchorMe = Vec2.ZERO;
  private Vec2 headOnAnchorOther = Vec2.ZERO;
  private double headOnFor = 0.0;
  private int yieldingTo = -1;
  private Vec2 yieldTarget = Vec2.ZERO;
  private Vec2 yieldDir = Vec2.ZERO;
  private double yieldUntil = Double.NEGATIVE_INFINITY;

  private boolean headingToGoal = true;
  private double dwellUntil = -1.0;
  private double lastScoreTime = Double.NaN;
  private int cycles = 0;
  private double cycleTimeSum = 0.0;
  private int timedCycles = 0;

  /**
   * @param pickup where it collects game pieces
   * @param goal where it scores
   * @param scoreRadius it can score from anywhere within this distance of the goal (m)
   * @param dwellSeconds time spent at the pickup and at the goal (s)
   */
  public CyclerBehavior(Vec2 pickup, Vec2 goal, double scoreRadius, double dwellSeconds) {
    this(pickup, goal, scoreRadius, dwellSeconds, false, 0.0);
  }

  private CyclerBehavior(
      Vec2 pickup,
      Vec2 goal,
      double scoreRadius,
      double dwellSeconds,
      boolean evasive,
      double reactionSec) {
    this.pickup = pickup;
    this.goal = goal;
    this.pickupRadius = 0.3;
    this.scoreRadius = scoreRadius;
    this.dwellSeconds = dwellSeconds;
    this.evasive = evasive;
    this.reactionSec = reactionSec;
  }

  /** Same cycle, but steers around robots in its path with a human reaction delay. */
  public static CyclerBehavior evader(Vec2 pickup, Vec2 goal, double scoreRadius, double dwellSeconds) {
    return evader(pickup, goal, scoreRadius, dwellSeconds, HUMAN_REACTION_SEC);
  }

  /**
   * @param reactionSec how stale its view of other robots is (s); 0 = superhuman, instant
   */
  public static CyclerBehavior evader(
      Vec2 pickup, Vec2 goal, double scoreRadius, double dwellSeconds, double reactionSec) {
    return new CyclerBehavior(pickup, goal, scoreRadius, dwellSeconds, true, reactionSec);
  }

  @Override
  public String name() {
    return evasive ? "Evader" : "Cycler";
  }

  @Override
  public ChassisCommand decide(double t, double dt, RobotBody me, SimWorld world) {
    if (t < dwellUntil) {
      stuckFor = 0.0;
      return ChassisCommand.STOP;
    }
    Vec2 dest = headingToGoal ? goal : pickup;
    double radius = headingToGoal ? scoreRadius : pickupRadius;
    double dist = me.position().distanceTo(dest);

    if (dist <= radius) {
      if (headingToGoal) {
        recordScore(t);
      }
      headingToGoal = !headingToGoal;
      dwellUntil = t + dwellSeconds;
      return ChassisCommand.STOP;
    }

    // Drive at the fastest speed that can still stop at the arrival radius:
    // v = sqrt(2 * a * remaining), using 80% of max accel for margin.
    if (!evasive) {
      ChassisCommand backOut = yieldHeadOn(t, dt, me, world);
      if (backOut != null) {
        return backOut;
      }
    }

    double remaining = dist - radius;
    double speed = Math.min(me.maxSpeed(), Math.sqrt(2.0 * 0.8 * me.maxAccel() * remaining));
    Vec2 aim = route(t, dt, me, world, dest);
    Vec2 dir = aim.minus(me.position()).normalized();
    if (evasive) {
      dir = evade(t, dt, me, world, aim, dir);
    }
    double turn = Angles.difference(dir.angle(), me.heading());
    return new ChassisCommand(dir.times(speed), 4.0 * turn);
  }

  /** Where to drive right now: straight at dest, or at the best gate if a wall is in the way. */
  private Vec2 route(double t, double dt, RobotBody me, SimWorld world, Vec2 dest) {
    var field = world.field();
    if (field.gates().isEmpty() || !field.pathBlocked(me.position(), dest)) {
      committedGate = -1;
      gateStuckFor = 0.0;
      return dest;
    }
    gateStuckFor = me.velocity().norm() < STUCK_SPEED ? gateStuckFor + dt : 0.0;
    if (gateStuckFor > STUCK_SECONDS && committedGate >= 0) {
      blockedGate = committedGate;
      blockedUntil = t + BLOCKED_SECONDS;
      gateStuckFor = 0.0;
      stuckFor = 0.0; // try another lane before bulldozing
    }
    List<Vec2> seen = new ArrayList<>();
    if (evasive) {
      for (Map.Entry<Integer, Vec2> e : world.positionsAt(t - reactionSec).entrySet()) {
        if (e.getKey() != me.id()) {
          seen.add(e.getValue());
        }
      }
    }
    int best = -1;
    double bestCost = Double.POSITIVE_INFINITY;
    for (int i = 0; i < field.gates().size(); i++) {
      double c = gateCost(me.position(), field.gates().get(i), dest, seen, i == committedGate);
      if (i == blockedGate && t < blockedUntil) {
        c += BLOCKED_PENALTY;
      }
      if (c < bestCost) {
        bestCost = c;
        best = i;
      }
    }
    committedGate = best;
    return field.gates().get(best);
  }

  /**
   * Cost of going through a gate (see class comment).
   *
   * @param seen robots this driver is watching for (empty for a Cycler)
   * @param committed true if this is the gate already chosen (no switch penalty)
   */
  static double gateCost(Vec2 pos, Vec2 gate, Vec2 dest, List<Vec2> seen, boolean committed) {
    double cost = pos.distanceTo(gate) + gate.distanceTo(dest);
    if (!committed) {
      cost += SWITCH_PENALTY;
    }
    for (Vec2 r : seen) {
      if (r.distanceTo(gate) < CROWDED_RADIUS) {
        cost += CROWDED_PENALTY;
      }
    }
    return cost;
  }

  private Vec2 evade(double t, double dt, RobotBody me, SimWorld world, Vec2 dest, Vec2 straight) {
    stuckFor = me.velocity().norm() < STUCK_SPEED ? stuckFor + dt : 0.0;
    if (stuckFor > STUCK_SECONDS) {
      bulldozeUntil = t + BULLDOZE_SECONDS;
      stuckFor = 0.0;
    }
    if (t < bulldozeUntil) {
      return straight;
    }
    // Other robots as this driver perceives them: where they were reactionSec ago.
    List<Vec2> obstacles = new ArrayList<>();
    for (Map.Entry<Integer, Vec2> e : world.positionsAt(t - reactionSec).entrySet()) {
      if (e.getKey() != me.id()) {
        obstacles.add(e.getValue());
      }
    }
    return Avoidance.steer(me.position(), dest, obstacles, AVOID_RADIUS, AVOID_GAIN);
  }

  /** Where this robot is driving right now (the goal or the pickup). */
  Vec2 destination() {
    return headingToGoal ? goal : pickup;
  }

  /**
   * Cycler only (audit M5): the head-on rule in the class comment. Returns the back-out command,
   * or null to drive on as usual.
   */
  private ChassisCommand yieldHeadOn(double t, double dt, RobotBody me, SimWorld world) {
    if (yieldingTo >= 0) {
      RobotBody other = null;
      for (SimWorld.Opponent o : world.opponents()) {
        if (o.body().id() == yieldingTo) {
          other = o.body();
        }
      }
      boolean passed = other == null
          || other.position().minus(me.position()).dot(yieldDir) < 0.0;
      if (!passed && t < yieldUntil) {
        Vec2 to = yieldTarget.minus(me.position());
        double d = to.norm();
        double speed = Math.min(me.maxSpeed(), Math.sqrt(2.0 * 0.8 * me.maxAccel() * d));
        return d < 0.05 ? ChassisCommand.STOP : new ChassisCommand(to.normalized().times(speed), 0.0);
      }
      yieldingTo = -1;
      headOnWith = -1;
      headOnFor = 0.0;
      return null;
    }
    // Find another Cycler stalled head-on with us inside a gap, if we are the lower id.
    var field = world.field();
    double touch = 2.0 * field.robotHalfSize() + TOUCH_GAP;
    int found = -1;
    Vec2 foundGate = null;
    CyclerBehavior foundBehavior = null;
    for (SimWorld.Opponent o : world.opponents()) {
      RobotBody b = o.body();
      if (b.id() <= me.id() || !(o.behavior() instanceof CyclerBehavior cb) || cb.evasive) {
        continue;
      }
      Vec2 n = b.position().minus(me.position()).normalized();
      if (b.position().distanceTo(me.position()) > touch
          || Contact.effort(me.desiredVelocity(), n, me.maxSpeed()) <= HEAD_ON_EFFORT
          || Contact.effort(b.desiredVelocity(), n.times(-1.0), b.maxSpeed()) <= HEAD_ON_EFFORT) {
        continue;
      }
      Vec2 mid = me.position().plus(b.position()).times(0.5);
      for (Vec2 g : field.gates()) {
        if (mid.distanceTo(g) <= GAP_RADIUS) {
          found = b.id();
          foundGate = g;
          foundBehavior = cb;
        }
      }
      if (found >= 0) {
        break;
      }
    }
    if (found < 0) {
      headOnWith = -1;
      headOnFor = 0.0;
      return null;
    }
    Vec2 otherPos = null;
    for (SimWorld.Opponent o : world.opponents()) {
      if (o.body().id() == found) {
        otherPos = o.body().position();
      }
    }
    if (found != headOnWith) {
      headOnWith = found;
      headOnFor = 0.0;
      headOnAnchorMe = me.position();
      headOnAnchorOther = otherPos;
    }
    headOnFor += dt;
    if (headOnFor < HEAD_ON_SECONDS - 1e-9) {
      return null;
    }
    // Stalled = neither got anywhere over the window. Net displacement, not the body's velocity:
    // in a crowd, collisions can keep the velocity high (and the position jittering by 0.01 m a
    // step) while the robot stands still.
    double limit = STUCK_SPEED * HEAD_ON_SECONDS;
    if (me.position().distanceTo(headOnAnchorMe) >= limit
        || otherPos.distanceTo(headOnAnchorOther) >= limit) {
      headOnFor = 0.0; // still moving through: start a new window from here
      headOnAnchorMe = me.position();
      headOnAnchorOther = otherPos;
      return null;
    }
    yieldingTo = found;
    yieldTarget = yieldPoint(me.position(), foundGate, foundBehavior.destination(), field);
    Vec2 u = foundGate.minus(me.position()).normalized();
    yieldDir = u.equals(Vec2.ZERO) ? foundGate.minus(foundBehavior.destination()).normalized() : u;
    yieldUntil = t + YIELD_MAX_SECONDS;
    return yieldHeadOn(t, dt, me, world);
  }

  /**
   * Where a Cycler backing out of a head-on stall drives to (see class comment).
   *
   * @param me its position
   * @param gate the gate it is stalled in
   * @param otherDest where the robot it yields to is driving
   */
  static Vec2 yieldPoint(Vec2 me, Vec2 gate, Vec2 otherDest, FieldConfig field) {
    Vec2 u = gate.minus(me).normalized();
    if (u.equals(Vec2.ZERO)) {
      u = gate.minus(otherDest).normalized(); // the other is headed for our side of the gate
    }
    Vec2 perp = new Vec2(-u.y(), u.x());
    Vec2 side = perp.dot(otherDest.minus(gate)) > 0.0 ? perp.times(-1.0) : perp;
    return field.clampInside(gate.minus(u.times(YIELD_BACK)).plus(side.times(YIELD_SIDE)));
  }

  private void recordScore(double t) {
    cycles++;
    if (!Double.isNaN(lastScoreTime)) {
      cycleTimeSum += t - lastScoreTime;
      timedCycles++;
    }
    lastScoreTime = t;
  }

  @Override
  public int cyclesCompleted() {
    return cycles;
  }

  @Override
  public double averageCycleSeconds() {
    return timedCycles == 0 ? Double.NaN : cycleTimeSum / timedCycles;
  }
}
