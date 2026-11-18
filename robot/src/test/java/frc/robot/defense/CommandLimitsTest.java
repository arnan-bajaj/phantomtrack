package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Vec2;

/** Safety rules: speed cap and acceleration limit. */
class CommandLimitsTest {
  private static final double TOL = 1e-9;

  private static ChassisCommand cmd(double vx, double vy, double omega) {
    return new ChassisCommand(new Vec2(vx, vy), omega);
  }

  @Test
  void speedCapShortensTheVectorAndKeepsItsDirection() {
    // |(3, 4)| = 5, cap 1.0 -> (3, 4) * 1/5 = (0.6, 0.8)
    ChassisCommand out = CommandLimits.cap(cmd(3, 4, 0), 1.0, 2.0);
    assertEquals(0.6, out.velocity().x(), TOL);
    assertEquals(0.8, out.velocity().y(), TOL);
    // Under the cap: unchanged.
    ChassisCommand slow = CommandLimits.cap(cmd(0.3, -0.4, 0), 1.0, 2.0);
    assertEquals(0.3, slow.velocity().x(), TOL);
    assertEquals(-0.4, slow.velocity().y(), TOL);
  }

  @Test
  void turnRateIsCappedBothWays() {
    assertEquals(2.0, CommandLimits.cap(cmd(0, 0, 5.0), 1.0, 2.0).omega(), TOL);
    assertEquals(-2.0, CommandLimits.cap(cmd(0, 0, -5.0), 1.0, 2.0).omega(), TOL);
    assertEquals(1.5, CommandLimits.cap(cmd(0, 0, 1.5), 1.0, 2.0).omega(), TOL);
  }

  @Test
  void accelerationLimitAllowsOnlyASmallStepPerLoop() {
    // From rest toward (1, 0) at 2 m/s^2 for 0.02 s: step = 0.04 m/s.
    ChassisCommand a = CommandLimits.limitAccel(cmd(0, 0, 0), cmd(1, 0, 0), 2.0, 6.0, 0.02);
    assertEquals(0.04, a.velocity().x(), TOL);
    assertEquals(0.0, a.velocity().y(), TOL);
    // The step is along the CHANGE: (0.5, 0) -> (0.5, 1.0): change (0, 1), step (0, 0.04).
    ChassisCommand b = CommandLimits.limitAccel(cmd(0.5, 0, 0), cmd(0.5, 1.0, 0), 2.0, 6.0, 0.02);
    assertEquals(0.5, b.velocity().x(), TOL);
    assertEquals(0.04, b.velocity().y(), TOL);
    // Slowing down is limited the same way: 1.0 -> 0 gives 0.96.
    ChassisCommand c = CommandLimits.limitAccel(cmd(1, 0, 0), cmd(0, 0, 0), 2.0, 6.0, 0.02);
    assertEquals(0.96, c.velocity().x(), TOL);
    // A change smaller than the step is applied in full: 0.99 -> 1.0.
    ChassisCommand d = CommandLimits.limitAccel(cmd(0.99, 0, 0), cmd(1, 0, 0), 2.0, 6.0, 0.02);
    assertEquals(1.0, d.velocity().x(), TOL);
  }

  @Test
  void turnAccelerationIsLimited() {
    // 6 rad/s^2 * 0.02 s = 0.12 rad/s per loop.
    assertEquals(0.12, CommandLimits.limitAccel(cmd(0, 0, 0), cmd(0, 0, 2), 2.0, 6.0, 0.02).omega(), TOL);
    assertEquals(0.88, CommandLimits.limitAccel(cmd(0, 0, 1), cmd(0, 0, -2), 2.0, 6.0, 0.02).omega(), TOL);
  }

  @Test
  void reachingOneMeterPerSecondTakesHalfASecond() {
    // 0.04 m/s per loop: 10 loops -> 0.4, 25 loops -> 1.0 (= 2 m/s^2 * 0.5 s), then it stays.
    ChassisCommand c = ChassisCommand.STOP;
    for (int i = 1; i <= 30; i++) {
      ChassisCommand next = CommandLimits.limitAccel(c, cmd(1, 0, 0), 2.0, 6.0, 0.02);
      assertTrue(next.velocity().minus(c.velocity()).norm() <= 0.04 + TOL);
      c = next;
      if (i == 10) {
        assertEquals(0.4, c.velocity().x(), TOL);
      }
    }
    assertEquals(1.0, c.velocity().x(), TOL);
  }

  @Test
  void zeroOrNegativeDtMeansNoChange() {
    ChassisCommand prev = cmd(0.3, 0.1, 0.2);
    ChassisCommand out = CommandLimits.limitAccel(prev, cmd(1, 1, 1), 2.0, 6.0, 0.0);
    assertEquals(0.3, out.velocity().x(), TOL);
    assertEquals(0.1, out.velocity().y(), TOL);
    assertEquals(0.2, out.omega(), TOL);
    ChassisCommand neg = CommandLimits.limitAccel(prev, cmd(1, 1, 1), 2.0, 6.0, -0.02);
    assertEquals(0.3, neg.velocity().x(), TOL);
  }

  @Test
  void nanOrInfinityBecomesStop() {
    assertSame(ChassisCommand.STOP, CommandLimits.cap(cmd(Double.NaN, 0, 0), 1.0, 2.0));
    assertSame(ChassisCommand.STOP, CommandLimits.cap(cmd(0, 0, Double.NaN), 1.0, 2.0));
    assertSame(ChassisCommand.STOP, CommandLimits.cap(null, 1.0, 2.0));
    assertSame(
        ChassisCommand.STOP,
        CommandLimits.limitAccel(cmd(0, 0, 0), cmd(Double.POSITIVE_INFINITY, 0, 0), 2.0, 6.0, 0.02));
    // A bad PREVIOUS command is treated as "we were stopped".
    ChassisCommand out = CommandLimits.limitAccel(cmd(Double.NaN, 0, 0), cmd(1, 0, 0), 2.0, 6.0, 0.02);
    assertEquals(0.04, out.velocity().x(), TOL);
    assertSame(ChassisCommand.STOP, CommandLimits.sanitize(cmd(0, Double.NaN, 0)));
  }
}
