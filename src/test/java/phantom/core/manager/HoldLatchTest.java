package phantom.core.manager;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.Test;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseContext;
import phantom.core.modes.DefenseMode;
import phantom.core.modes.DefenseTarget;
import phantom.core.modes.HoldGroundMode;
import phantom.core.tracking.TrackedRobot;

/** Audit U1: hold pushes only on opponents closing on us or on the goal, with hysteresis. */
class HoldLatchTest {
  private static final double TOL = 1e-9;
  private static final double DT = 0.02;
  private static final Vec2 SELF = new Vec2(5, 4);
  private static final Vec2 OPP = new Vec2(5.9, 4);
  private static final Vec2 GOAL = new Vec2(0, 4);

  /**
   * Opponent at (5.9, 4) moving (vx, 0); we are at (5, 4), goal (0, 4):
   * n = unit((0.9, 0)) = (1, 0), g = unit((-5.9, 0)) = (-1, 0)
   * closing = max(-vx, -vx) = -vx. So vx = 0.30 gives closing = -0.30 m/s (driving away).
   */
  private static List<TrackedRobot> oppMoving(double vx, double t) {
    return List.of(new TrackedRobot(0, OPP, new Vec2(vx, 0), t, 1.0));
  }

  @Test
  void closingSpeedIsTheLargerOfClosingOnUsAndOnTheGoal() {
    // self (5,4), opp (6,4), goal (6,0): n = (1,0), g = unit((0,-4)) = (0,-1)
    // v = (0.5,-1): on us = -(0.5*1 + -1*0) = -0.5, on goal = 0.5*0 + (-1)(-1) = 1.0 -> 1.0
    assertEquals(1.0, HoldGroundMode.closingSpeed(new Vec2(5, 4), new Vec2(6, 4),
        new Vec2(0.5, -1), new Vec2(6, 0)), TOL);
    // v = (-0.3,0): on us = -(-0.3) = 0.3, on goal = 0 -> 0.3
    assertEquals(0.3, HoldGroundMode.closingSpeed(new Vec2(5, 4), new Vec2(6, 4),
        new Vec2(-0.3, 0), new Vec2(6, 0)), TOL);
    // stalled pushing match: v = 0 -> 0 (keeps the hold: 0 > -0.25)
    assertEquals(0.0, HoldGroundMode.closingSpeed(SELF, OPP, Vec2.ZERO, GOAL), TOL);
    // opponent on top of us and on the goal: both unit vectors are ZERO -> max(0, 0) = 0, not NaN
    double c = HoldGroundMode.closingSpeed(GOAL, GOAL, new Vec2(1, 0), GOAL);
    assertEquals(0.0, c, TOL);
    assertTrue(Double.isFinite(c));
  }

  @Test
  void steadyLeavingReleasesAfterExactlyPointTwoSeconds() {
    // closing = -0.30 < -0.25 from t = 0: below since 0; released at the first update with
    // t - 0 >= 0.2, i.e. update 10 (t = 0.20). Updates 0..9 (t = 0 .. 0.18) still allow hold.
    HoldLatch latch = new HoldLatch();
    for (int i = 0; i <= 9; i++) {
      assertTrue(latch.update(i * DT, SELF, oppMoving(0.30, i * DT), GOAL).isEmpty(), "update " + i);
    }
    assertEquals(Set.of(0), latch.update(10 * DT, SELF, oppMoving(0.30, 10 * DT), GOAL));
    assertEquals(1, latch.switches());
    // back on as soon as closing > -0.10: vx = 0.05 -> closing -0.05 > -0.10
    assertTrue(latch.update(11 * DT, SELF, oppMoving(0.05, 11 * DT), GOAL).isEmpty());
    assertEquals(2, latch.switches());
  }

  @Test
  void noisyVelocityAroundMinusPointTwoFiveDoesNotToggleEveryLoop() {
    // Closing alternates -0.30, -0.20, -0.30, ... (vx = 0.30, 0.20): a noisy estimate of an
    // opponent drifting away at about 0.25 m/s. 100 updates (2 s).
    //   Hysteresis: never 0.2 s in a row below -0.25 (every other update is -0.20 >= -0.25 and
    //   restarts the timer) -> hold stays allowed, 0 switches.
    //   An instant rule "hold iff closing >= -0.25", starting from allowed, would flip on every
    //   update: update 0 (-0.30: allowed -> off), update 1 (-0.20: on), ... -> 100 switches.
    HoldLatch latch = new HoldLatch();
    int instantSwitches = 0;
    boolean instantPrev = true;
    for (int i = 0; i < 100; i++) {
      double vx = i % 2 == 0 ? 0.30 : 0.20;
      assertTrue(latch.update(i * DT, SELF, oppMoving(vx, i * DT), GOAL).isEmpty(), "update " + i);
      boolean instant = -vx >= HoldLatch.OFF_BELOW;
      instantSwitches += instant != instantPrev ? 1 : 0;
      instantPrev = instant;
    }
    assertEquals(0, latch.switches());
    assertEquals(100, instantSwitches);

    // Once released (11 updates at -0.30: t = 2.00 .. 2.20, released at t - 2.00 >= 0.2), noise
    // between -0.30 and -0.15 never rises above -0.10 -> stays released: 1 switch in total.
    for (int i = 100; i <= 110; i++) {
      latch.update(i * DT, SELF, oppMoving(0.30, i * DT), GOAL);
    }
    assertEquals(1, latch.switches());
    for (int i = 111; i < 211; i++) {
      double vx = i % 2 == 0 ? 0.30 : 0.15;
      assertEquals(Set.of(0), latch.update(i * DT, SELF, oppMoving(vx, i * DT), GOAL), "update " + i);
    }
    assertEquals(1, latch.switches());
  }

  @Test
  void aStalledPushIsNeverReleasedAndANewTrackStartsAllowed() {
    HoldLatch latch = new HoldLatch();
    for (int i = 0; i < 100; i++) { // v = 0 -> closing 0 > -0.25
      assertTrue(latch.update(i * DT, SELF, oppMoving(0.0, i * DT), GOAL).isEmpty());
    }
    for (int i = 100; i <= 110; i++) { // released at i = 110 (t - 2.00 = 0.20)
      latch.update(i * DT, SELF, oppMoving(0.5, i * DT), GOAL);
    }
    assertEquals(Set.of(0), latch.update(111 * DT, SELF, oppMoving(0.5, 111 * DT), GOAL));
    // track 0 lost for one update -> its state is dropped; it comes back allowed even though it
    // is still leaving (the timer starts again)
    latch.update(112 * DT, SELF, List.of(), GOAL);
    assertTrue(latch.update(113 * DT, SELF, oppMoving(0.5, 113 * DT), GOAL).isEmpty());
  }

  /** Inner mode that always targets (2,4) against opponent 0. */
  private static final DefenseMode RETREAT = new DefenseMode() {
    public String name() {
      return "Retreat";
    }

    public Optional<DefenseTarget> compute(DefenseContext ctx) {
      return Optional.of(new DefenseTarget(new Vec2(2, 4), Math.PI, Vec2.ZERO, 0));
    }
  };

  @Test
  void holdDoesNotPushOnAReleasedOpponent() {
    // Same geometry as PinAndHoldTest.holdPushesBackWhenTouchingAndGoalSide: gap 0, goal-side,
    // so hold would push to (5.3, 4). With opponent 0 released, the inner target (2, 4) is kept.
    var field = new FieldConfig(20, 20, GOAL, List.of(), 0.45);
    var mode = new HoldGroundMode(RETREAT, 0.3, 4.5);
    var r = oppMoving(0.5, 0.0);
    var held = mode.compute(new DefenseContext(0.0, new Pose(SELF, 0), Vec2.ZERO, r, field, 0))
        .orElseThrow();
    assertEquals(5.3, held.position().x(), TOL);
    var released = mode.compute(new DefenseContext(0.0, new Pose(SELF, 0), Vec2.ZERO, r, field, 0,
        Set.of(0))).orElseThrow();
    assertEquals(2.0, released.position().x(), TOL);
    assertFalse(mode.holdTarget(new DefenseContext(0.0, new Pose(SELF, 0), Vec2.ZERO, r, field, 0,
        Set.of(0)), Optional.of(released)).isPresent());
  }
}
