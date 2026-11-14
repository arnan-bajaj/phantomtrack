package phantom.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.field.Obstacle;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseTarget;

/** Obstacle geometry, field clamping, gate routing (ours and opponents'). */
class ObstacleTest {
  private static final double TOL = 1e-9;
  // Rectangle (2,2)-(4,3); robot radius 0.5 -> grown to x (1.5, 4.5), y (1.5, 3.5)
  private static final Obstacle BOX = Obstacle.of(2, 2, 4, 3);

  @Test
  void blocksUsesTheGrownRectangle() {
    assertTrue(BOX.blocks(new Vec2(3, 2.5), 0.5));
    assertFalse(BOX.blocks(new Vec2(1.4, 2.5), 0.5)); // 1.4 < 1.5
    assertTrue(BOX.blocks(new Vec2(1.6, 3.4), 0.5)); // inside the grown corner
  }

  @Test
  void pushOutTakesTheShortestWay() {
    // (3,1.7): left 1.5, right 1.5, down 1.7 - 1.5 = 0.2, up 3.5 - 1.7 = 1.8 -> down -> (3,1.5)
    assertEquals(new Vec2(3, 1.5), BOX.pushOut(new Vec2(3, 1.7), 0.5));
    // (1.6,2.5): left 1.6 - 1.5 = 0.1 is smallest -> (1.5,2.5)
    assertEquals(new Vec2(1.5, 2.5), BOX.pushOut(new Vec2(1.6, 2.5), 0.5));
    // already clear: unchanged
    assertEquals(new Vec2(0, 0), BOX.pushOut(new Vec2(0, 0), 0.5));
  }

  @Test
  void blocksPathSlabMethod() {
    assertTrue(BOX.blocksPath(new Vec2(0, 2.5), new Vec2(6, 2.5), 0.5)); // straight through
    assertFalse(BOX.blocksPath(new Vec2(0, 4), new Vec2(6, 4), 0.5)); // y = 4 >= 3.5, passes above
    assertFalse(BOX.blocksPath(new Vec2(0, 0), new Vec2(1, 1), 0.5)); // stops short
    assertFalse(BOX.blocksPath(new Vec2(5, 0), new Vec2(5, 6), 0.5)); // x = 5 outside, parallel
    // y = x: x in (1.5,4.5) -> t in (0.25,0.75); y in (1.5,3.5) -> t in (0.25,0.583) -> overlap
    assertTrue(BOX.blocksPath(new Vec2(0, 0), new Vec2(6, 6), 0.5));
  }

  @Test
  void fieldClampPushesOutOfWalls() {
    // choke wall (8.6,0)-(9.0,2.8), r = 0.45 -> grown x (8.15, 9.45), y (-0.45, 3.25)
    // (8.7,1.0): left 8.7 - 8.15 = 0.55, right 9.45 - 8.7 = 0.75, down 1.45, up 2.25 -> (8.15,1.0)
    Vec2 p = FieldConfig.choke().clampInside(new Vec2(8.7, 1.0));
    assertEquals(8.15, p.x(), TOL);
    assertEquals(1.0, p.y(), TOL);
    // open field: same point untouched
    assertEquals(new Vec2(8.7, 1.0), FieldConfig.placeholder().clampInside(new Vec2(8.7, 1.0)));
  }

  @Test
  void gateCostByHand() {
    Vec2 pos = new Vec2(0, 0);
    Vec2 gate = new Vec2(3, 4);
    Vec2 dest = new Vec2(6, 0);
    // |pos-gate| = 5, |gate-dest| = 5 -> 10, not the committed gate: +1.5 -> 11.5
    assertEquals(11.5, CyclerBehavior.gateCost(pos, gate, dest, List.of(), false), TOL);
    // a robot 1 m from the gate (< 2 m): +4 -> 15.5
    assertEquals(15.5, CyclerBehavior.gateCost(pos, gate, dest, List.of(new Vec2(3, 5)), false), TOL);
    // committed, robot 3 m away: 10
    assertEquals(10.0, CyclerBehavior.gateCost(pos, gate, dest, List.of(new Vec2(3, 7)), true), TOL);
  }

  @Test
  void defenderRoutesThroughTheGate() {
    var s = SimSession.standard(Scenarios.OpponentKind.CYCLER, SimSession.Sensor.PERFECT, 1, Scenarios.Layout.CHOKE);
    var target = new DefenseTarget(new Vec2(5, 0.6), 0.5, new Vec2(1, 1), 2);
    // (11,4.035) -> (5,0.6) crosses the lower wall -> go to the only gate (8.8, 4.035), no feedforward
    var routed = s.assist().routeAroundObstacles(new Pose(11, 4.035, 0), target);
    assertEquals(8.8, routed.position().x(), TOL);
    assertEquals(4.035, routed.position().y(), TOL);
    assertEquals(Vec2.ZERO, routed.velocityFeedforward());
    assertEquals(0.5, routed.heading(), TOL);
    // clear path: unchanged
    var clear = new DefenseTarget(new Vec2(12, 4), 0, Vec2.ZERO, 2);
    assertEquals(clear, s.assist().routeAroundObstacles(new Pose(11, 4.035, 0), clear));
  }

  @Test
  void cyclerScoresThroughTheChokeWithoutEnteringWalls() {
    var field = FieldConfig.choke();
    var pickup = field.opponentPickups().get(0);
    var opp = RobotBody.typical(0, new Pose(pickup, 0));
    var world = new SimWorld(field, RobotBody.typical(100, Scenarios.parkedOutOfTheWay(field)),
        List.of(new SimWorld.Opponent(opp, new CyclerBehavior(pickup, field.defendedGoal(), 1.5, 0.5))));
    for (int i = 0; i < 1500; i++) { // 30 s
      world.step(ChassisCommand.STOP, 0.02);
      for (Obstacle o : field.obstacles()) {
        assertFalse(o.blocks(opp.position(), field.robotHalfSize() - 1e-6), "inside a wall at " + opp.position());
      }
    }
    assertTrue(world.opponents().get(0).behavior().cyclesCompleted() >= 3);
  }

  @Test
  void evaderTakesTheOpenLane() {
    // two lanes: gates (8.8,2.0) and (8.8,6.07). Evader at (6,5.5) heading to goal (13,4.035):
    // upper = 2.857 + 4.667 = 7.52, lower = 4.482 + 4.667 = 9.15 -> upper, unless a robot sits
    // within 2 m of it: 7.52 + 4 = 11.52 > 9.15 -> lower.
    var field = FieldConfig.twoLanes();
    Pose defenderAtUpperGate = new Pose(9.6, 6.07, Math.PI); // 0.8 m from the upper gate
    Vec2 start = new Vec2(6, 5.5);
    var evader = RobotBody.typical(0, new Pose(start, 0));
    var cycler = RobotBody.typical(0, new Pose(start, 0));
    var w1 = new SimWorld(field, RobotBody.typical(100, defenderAtUpperGate),
        List.of(new SimWorld.Opponent(evader, CyclerBehavior.evader(new Vec2(1.5, 6.9), field.defendedGoal(), 1.5, 0.0))));
    var w2 = new SimWorld(field, RobotBody.typical(100, defenderAtUpperGate),
        List.of(new SimWorld.Opponent(cycler, new CyclerBehavior(new Vec2(1.5, 6.9), field.defendedGoal(), 1.5, 0.0))));
    for (int i = 0; i < 50; i++) { // 1 s
      w1.step(ChassisCommand.STOP, 0.02);
      w2.step(ChassisCommand.STOP, 0.02);
    }
    assertTrue(evader.position().y() < 5.0, "evader should head for the lower lane: " + evader.position());
    assertTrue(cycler.position().y() > 5.4, "cycler takes the shorter upper lane: " + cycler.position());
  }

  @Test
  void opponentsDoNotGridlockInALane() throws IOException {
    // Before the fix, two Cyclers met head-on in a lane and stalled for the rest of the match
    var s = SimMain.run(new SimMain.Setup("lm auto", 3, true, true, SimSession.Sensor.PERFECT, 1, -1, true)
        .withLayout(Scenarios.Layout.TWO_LANES), Scenarios.OpponentKind.CYCLER, 180, null);
    assertTrue(SimMain.totalCycles(s) > 55, "total cycles " + SimMain.totalCycles(s));
  }
}
