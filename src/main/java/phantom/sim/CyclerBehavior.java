package phantom.sim;

import java.util.ArrayList;
import java.util.List;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Vec2;

/**
 * Drives pickup -> goal -> pickup -> ... forever and counts a cycle each time it reaches scoring
 * range of the goal.
 *
 * <ul>
 *   <li><b>Cycler</b> ignores defenders and drives straight through them.
 *   <li><b>Evader</b> steers around robots in its path ({@link Avoidance}). If it is stuck (under
 *       0.3 m/s for 1 s) it gives up dodging and pushes straight for 1 s.
 * </ul>
 */
public final class CyclerBehavior implements OpponentBehavior {
  private final Vec2 pickup;
  private final Vec2 goal;
  private final double pickupRadius;
  private final double scoreRadius;
  private final double dwellSeconds;
  private final boolean evasive;

  // Evader tuning
  private static final double AVOID_RADIUS = 3.0;
  private static final double AVOID_GAIN = 1.5;
  private static final double STUCK_SPEED = 0.3;
  private static final double STUCK_SECONDS = 1.0;
  private static final double BULLDOZE_SECONDS = 1.0;
  private double stuckFor = 0.0;
  private double bulldozeUntil = -1.0;

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
    this(pickup, goal, scoreRadius, dwellSeconds, false);
  }

  private CyclerBehavior(
      Vec2 pickup, Vec2 goal, double scoreRadius, double dwellSeconds, boolean evasive) {
    this.pickup = pickup;
    this.goal = goal;
    this.pickupRadius = 0.3;
    this.scoreRadius = scoreRadius;
    this.dwellSeconds = dwellSeconds;
    this.evasive = evasive;
  }

  /** Same cycle, but steers around robots in its path. */
  public static CyclerBehavior evader(Vec2 pickup, Vec2 goal, double scoreRadius, double dwellSeconds) {
    return new CyclerBehavior(pickup, goal, scoreRadius, dwellSeconds, true);
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
    double remaining = dist - radius;
    double speed = Math.min(me.maxSpeed(), Math.sqrt(2.0 * 0.8 * me.maxAccel() * remaining));
    Vec2 dir = dest.minus(me.position()).normalized();
    if (evasive) {
      dir = evade(t, dt, me, world, dest, dir);
    }
    double turn = Angles.difference(dir.angle(), me.heading());
    return new ChassisCommand(dir.times(speed), 4.0 * turn);
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
    List<Vec2> obstacles = new ArrayList<>();
    obstacles.add(world.self().position());
    for (SimWorld.Opponent o : world.opponents()) {
      if (o.body() != me) {
        obstacles.add(o.body().position());
      }
    }
    return Avoidance.steer(me.position(), dest, obstacles, AVOID_RADIUS, AVOID_GAIN);
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
