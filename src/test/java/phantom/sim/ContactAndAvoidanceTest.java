package phantom.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

class ContactAndAvoidanceTest {
  private static final double TOL = 1e-9;

  // ---------------- Contact formulas ----------------

  @Test
  void effortIsNormalizedAndClamped() {
    Vec2 n = new Vec2(1, 0);
    // (4.5,0).(1,0) / 4.5 = 1
    assertEquals(1.0, Contact.effort(new Vec2(4.5, 0), n, 4.5), TOL);
    // (2.25,3).(1,0) / 4.5 = 0.5  (sideways motion doesn't count)
    assertEquals(0.5, Contact.effort(new Vec2(2.25, 3), n, 4.5), TOL);
    // (-9,0).(1,0) / 4.5 = -2 -> clamped to -1
    assertEquals(-1.0, Contact.effort(new Vec2(-9, 0), n, 4.5), TOL);
  }

  @Test
  void equalPushStalls() {
    // net = 1 - 1 = 0 -> v_n = 2.0 * 0 / 2 = 0, share_B = 0.5 + 0/4 = 0.5
    assertEquals(0.0, Contact.contactSpeed(1, 1), TOL);
    assertEquals(0.5, Contact.overlapShareB(1, 1), TOL);
  }

  @Test
  void pushingAnIdleRobot() {
    // A full, B idle: net = 1 -> v_n = 2.0 * 1 / 2 = 1.0 m/s, share_B = 0.5 + 1/4 = 0.75
    assertEquals(1.0, Contact.contactSpeed(1, 0), TOL);
    assertEquals(0.75, Contact.overlapShareB(1, 0), TOL);
  }

  @Test
  void bracingRobotWinsAgainstIdleOne() {
    // A idle, B pushing into A: net = 0 - 1 = -1 -> v_n = -1.0 (both move toward A's side)
    // share_B = 0.5 - 1/4 = 0.25 (A gives up 75%)
    assertEquals(-1.0, Contact.contactSpeed(0, 1), TOL);
    assertEquals(0.25, Contact.overlapShareB(0, 1), TOL);
  }

  // ---------------- Contact in the sim ----------------

  private static SimWorld twoRobots(Pose a, Pose b, ChassisCommand bCmd) {
    var field = FieldConfig.placeholder();
    OpponentBehavior scripted =
        new OpponentBehavior() {
          public String name() {
            return "Scripted";
          }

          public ChassisCommand decide(double t, double dt, RobotBody me, SimWorld w) {
            return bCmd;
          }

          public int cyclesCompleted() {
            return 0;
          }

          public double averageCycleSeconds() {
            return Double.NaN;
          }
        };
    return new SimWorld(
        field,
        RobotBody.typical(100, a),
        List.of(new SimWorld.Opponent(RobotBody.typical(0, b), scripted)));
  }

  @Test
  void headOnEqualPushStallsInSim() {
    // A at x=7.6 drives +x, B at x=8.5 drives -x, both full speed: they meet and stall.
    var w = twoRobots(new Pose(7.6, 4, 0), new Pose(8.5, 4, Math.PI), new ChassisCommand(new Vec2(-4.5, 0), 0));
    for (int i = 0; i < 150; i++) { // 3 s
      w.step(new ChassisCommand(new Vec2(4.5, 0), 0), 0.02);
    }
    double mid = (w.self().position().x() + w.opponents().get(0).body().position().x()) / 2;
    assertEquals(8.05, mid, 0.05); // the pair's midpoint barely moves
  }

  @Test
  void pushingAnIdleRobotMovesIt() {
    // A drives +x into a parked B for 3 s: B gets shoved well over 1 m.
    var w = twoRobots(new Pose(7.0, 4, 0), new Pose(8.0, 4, 0), ChassisCommand.STOP);
    for (int i = 0; i < 150; i++) {
      w.step(new ChassisCommand(new Vec2(4.5, 0), 0), 0.02);
    }
    assertTrue(w.opponents().get(0).body().position().x() > 9.0);
    assertTrue(w.selfContactSeconds() > 1.0);
  }

  // ---------------- Avoidance ----------------

  @Test
  void noObstaclesMeansStraightAtDestination() {
    Vec2 s = Avoidance.steer(new Vec2(0, 0), new Vec2(10, 0), List.of(), 3.0, 1.5);
    assertEquals(1.0, s.x(), TOL);
    assertEquals(0.0, s.y(), TOL);
  }

  @Test
  void obstacleAheadOnLeftSteersRight() {
    // pos (0,0), dest (10,0): dir = (1,0), perp = (0,1)
    // obstacle (2,0.5): d = sqrt(4.25) = 2.0615528, along = 2, rel.perp = 0.5 >= 0 -> side +1
    // strength = 1.5 * (1/2.0615528 - 1/3) = 1.5 * (0.4850713 - 0.3333333) = 0.2276069
    // push = -1 * 0.2276069 * (0,1) = (0, -0.2276069)
    // steer = unit(1, -0.2276069) = (1, -0.2276069) / 1.0255754 = (0.9750624, -0.2219309)
    Vec2 s = Avoidance.steer(new Vec2(0, 0), new Vec2(10, 0), List.of(new Vec2(2, 0.5)), 3.0, 1.5);
    assertEquals(0.9750624, s.x(), 1e-6);
    assertEquals(-0.2219309, s.y(), 1e-6);
  }

  @Test
  void symmetricObstaclesCancel() {
    // (2,0.5) pushes right by 0.2276, (2,-0.5) pushes left by 0.2276 -> straight ahead
    Vec2 s =
        Avoidance.steer(
            new Vec2(0, 0), new Vec2(10, 0), List.of(new Vec2(2, 0.5), new Vec2(2, -0.5)), 3.0, 1.5);
    assertEquals(1.0, s.x(), TOL);
    assertEquals(0.0, s.y(), TOL);
  }

  @Test
  void ignoresObstaclesBehindFarOrPastTheDestination() {
    List<Vec2> obs = List.of(new Vec2(-1, 0.2), new Vec2(5, 0.2), new Vec2(2.5, 0.2));
    // behind (along < 0), too far (d = 5.004 > 3), past a destination 2 m away (along 2.5 >= 2)
    Vec2 s = Avoidance.steer(new Vec2(0, 0), new Vec2(2, 0), obs, 3.0, 1.5);
    assertEquals(1.0, s.x(), TOL);
    assertEquals(0.0, s.y(), TOL);
  }

  @Test
  void atDestinationReturnsZero() {
    assertEquals(Vec2.ZERO, Avoidance.steer(new Vec2(3, 3), new Vec2(3, 3), List.of(), 3.0, 1.5));
  }

  @Test
  void worldRemembersWhereRobotsWere() {
    // Our robot drives +x at a constant 2 m/s (after the accel limit is reached it stays there).
    var w = twoRobots(new Pose(2, 4, 0), new Pose(14, 7, 0), ChassisCommand.STOP);
    for (int i = 0; i < 100; i++) { // 2 s
      w.step(new ChassisCommand(new Vec2(2, 0), 0), 0.02);
    }
    double now = w.timeSec();
    double xNow = w.positionsAt(now).get(100).x();
    // 0.24 s = 12 steps of 20 ms. At 2 m/s, 0.24 s ago it was 2 * 0.24 = 0.48 m behind
    double xAgo = w.positionsAt(now - 0.24).get(100).x();
    assertEquals(0.48, xNow - xAgo, 1e-6);
    // 0.25 s falls between steps (12.5): we get the snapshot at or before it, 13 steps = 0.52 m
    assertEquals(0.52, xNow - w.positionsAt(now - 0.25).get(100).x(), 1e-6);
    // Asking for a time before the 2 s history returns the oldest record, never null
    assertTrue(w.positionsAt(-100).containsKey(100));
  }

  @Test
  void evaderDodgesAParkedRobotThatACyclerHits() {
    var field = FieldConfig.placeholder();
    Vec2 pickup = new Vec2(3.0, 4.035);
    // Parked robot sits right on the straight line from pickup to goal.
    Pose parked = new Pose(8.0, 4.035, 0);

    var cyclerWorld =
        new SimWorld(field, RobotBody.typical(100, parked),
            List.of(new SimWorld.Opponent(RobotBody.typical(0, new Pose(pickup, 0)),
                new CyclerBehavior(pickup, field.defendedGoal(), 1.5, 0.5))));
    var evaderWorld =
        new SimWorld(field, RobotBody.typical(100, parked),
            List.of(new SimWorld.Opponent(RobotBody.typical(0, new Pose(pickup, 0)),
                CyclerBehavior.evader(pickup, field.defendedGoal(), 1.5, 0.5))));
    for (int i = 0; i < 1500; i++) { // 30 s, parked robot holds still (STOP)
      cyclerWorld.step(ChassisCommand.STOP, 0.02);
      evaderWorld.step(ChassisCommand.STOP, 0.02);
    }
    assertTrue(cyclerWorld.selfContactSeconds() > 1.0, "cycler should hit the parked robot");
    assertTrue(evaderWorld.selfContactSeconds() < 0.2, "evader touched for " + evaderWorld.selfContactSeconds());
    assertTrue(evaderWorld.opponents().get(0).behavior().cyclesCompleted() >= 3);
  }
}
