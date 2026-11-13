package phantom.sim;

import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Vec2;

/**
 * Drives pickup -> goal -> pickup -> ... forever, ignoring defenders (it just pushes through).
 * Counts a cycle each time it reaches scoring range of the goal.
 */
public final class CyclerBehavior implements OpponentBehavior {
  private final Vec2 pickup;
  private final Vec2 goal;
  private final double pickupRadius;
  private final double scoreRadius;
  private final double dwellSeconds;

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
    this.pickup = pickup;
    this.goal = goal;
    this.pickupRadius = 0.3;
    this.scoreRadius = scoreRadius;
    this.dwellSeconds = dwellSeconds;
  }

  @Override
  public String name() {
    return "Cycler";
  }

  @Override
  public ChassisCommand decide(double t, double dt, RobotBody me, SimWorld world) {
    if (t < dwellUntil) {
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
    double turn = Angles.difference(dir.angle(), me.heading());
    return new ChassisCommand(dir.times(speed), 4.0 * turn);
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
