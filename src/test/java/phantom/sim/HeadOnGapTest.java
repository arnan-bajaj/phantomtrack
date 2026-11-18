package phantom.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.field.Obstacle;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/**
 * Audit M5: two Cyclers meeting head-on inside a gap. After 1.0 s stalled, the lower id backs out
 * until the other has passed. A Cycler never backs out from our robot.
 */
class HeadOnGapTest {
  private static final double TOL = 1e-9;
  private static final double DT = 0.02;

  /**
   * Hand-built gap: 16.54 x 8.07 field, one wall at x 8.6..9.0 with a 2.0 m gap at y 3.035..5.035
   * (gate (8.8, 4.035)). Robot half size 0.45 -> the grown walls cover x 8.15..9.45.
   */
  static final Vec2 GATE = new Vec2(8.8, 4.035);

  static FieldConfig gapField() {
    return new FieldConfig("Gap", 16.54, 8.07, new Vec2(13.0, 4.035),
        List.of(new Vec2(3.0, 4.035)), 0.45,
        List.of(Obstacle.of(8.6, 0.0, 9.0, 3.035), Obstacle.of(8.6, 5.035, 9.0, 8.07)),
        List.of(GATE));
  }

  /**
   * East-bound Cycler with body id eastId starts at (6, 4.035) driving to (13, 4.035); west-bound
   * Cycler with body id westId starts at (11, 4.035) driving to (3, 4.035) (its "goal" is the
   * west end, so it heads west first). Both are on the gate's line, so they meet exactly head-on:
   * both full speed, equal distance (2.8 m) to the gate -> they touch with centers at 8.05 and
   * 8.95, both inside the grown wall's x range. Our robot is parked in the far corner.
   */
  static SimWorld headOn(int eastId, int westId) {
    FieldConfig f = gapField();
    RobotBody self = RobotBody.typical(100, Scenarios.parkedOutOfTheWay(f));
    RobotBody east = RobotBody.typical(eastId, new Pose(new Vec2(6.0, 4.035), 0.0));
    RobotBody west = RobotBody.typical(westId, new Pose(new Vec2(11.0, 4.035), Math.PI));
    var eastB = new CyclerBehavior(new Vec2(3.0, 4.035), new Vec2(13.0, 4.035), 0.3, 0.5);
    var westB = new CyclerBehavior(new Vec2(14.0, 4.035), new Vec2(3.0, 4.035), 0.3, 0.5);
    List<SimWorld.Opponent> opps = eastId < westId
        ? List.of(new SimWorld.Opponent(east, eastB), new SimWorld.Opponent(west, westB))
        : List.of(new SimWorld.Opponent(west, westB), new SimWorld.Opponent(east, eastB));
    return new SimWorld(f, self, opps);
  }

  static RobotBody body(SimWorld w, int id) {
    for (SimWorld.Opponent o : w.opponents()) {
      if (o.body().id() == id) {
        return o.body();
      }
    }
    throw new IllegalArgumentException("no body " + id);
  }

  @Test
  void yieldPointIsBehindTheGateAndAwayFromTheOtherRobotsDestination() {
    // A at (8.05, 4.035), gate (8.8, 4.035): u = (0.75, 0)/0.75 = (1, 0), perp = (-u.y, u.x) = (0, 1)
    // other's destination (3, 1.2): (dest - gate) = (-5.8, -2.835), perp . that = -2.835 < 0
    //   -> step to +perp: Y = gate - 1.5 u + 1.2 perp = (8.8 - 1.5, 4.035 + 1.2) = (7.3, 5.235)
    FieldConfig f = gapField();
    Vec2 a = new Vec2(8.05, 4.035);
    Vec2 y1 = CyclerBehavior.yieldPoint(a, GATE, new Vec2(3.0, 1.2), f);
    assertEquals(7.3, y1.x(), TOL);
    assertEquals(5.235, y1.y(), TOL);
    // destination (3, 6.9): perp . (-5.8, 2.865) = 2.865 > 0 -> -perp: Y = (7.3, 4.035 - 1.2) = (7.3, 2.835)
    Vec2 y2 = CyclerBehavior.yieldPoint(a, GATE, new Vec2(3.0, 6.9), f);
    assertEquals(7.3, y2.x(), TOL);
    assertEquals(2.835, y2.y(), TOL);
    // destination straight through (3, 4.035): perp . (-5.8, 0) = 0 -> tie goes to +perp: (7.3, 5.235)
    Vec2 y3 = CyclerBehavior.yieldPoint(a, GATE, new Vec2(3.0, 4.035), f);
    assertEquals(7.3, y3.x(), TOL);
    assertEquals(5.235, y3.y(), TOL);
    // A exactly on the gate: unit(gate - A) = unit(0) = ZERO, so u falls back to
    // unit(gate - otherDest) = (5.8, 0)/5.8 = (1, 0) (the other is headed for A's side) -> same as y3.
    Vec2 y4 = CyclerBehavior.yieldPoint(GATE, GATE, new Vec2(3.0, 4.035), f);
    assertEquals(7.3, y4.x(), TOL);
    assertEquals(5.235, y4.y(), TOL);
    assertTrue(y4.isFinite());
  }

  @Test
  void lowerIdBacksOutAndBothGetThrough() {
    SimWorld w = headOn(0, 1); // east-bound robot has the lower id -> it backs out
    RobotBody east = body(w, 0);
    RobotBody west = body(w, 1);
    double eastMinXAfterStall = Double.POSITIVE_INFINITY;
    double westMaxX = Double.NEGATIVE_INFINITY;
    double westThroughAt = Double.NaN;
    for (int i = 0; i < (int) Math.round(30.0 / DT); i++) {
      w.step(ChassisCommand.STOP, DT);
      double t = w.timeSec();
      if (t > 3.0) {
        eastMinXAfterStall = Math.min(eastMinXAfterStall, east.position().x());
      }
      if (t > 3.0 && Double.isNaN(westThroughAt)) {
        westMaxX = Math.max(westMaxX, west.position().x());
      }
      if (Double.isNaN(westThroughAt) && west.position().x() < 7.0) {
        westThroughAt = t;
      }
    }
    // Contact at about t = 2.8 / 4.5 + the acceleration time ~ 1 s; the stall must last 1 s
    // first, so the west-bound robot is through the gap well before 10 s.
    assertTrue(westThroughAt < 10.0, "west-bound robot through the gap at " + westThroughAt);
    // The east-bound (lower id) robot is the one that backed out: it went back past the wall's
    // grown edge (8.15) toward its yield point (x 7.3).
    assertTrue(eastMinXAfterStall < 7.6, "east-bound min x " + eastMinXAfterStall);
    // The west-bound (higher id) robot never gave ground: from its stall at x ~8.95 it only went
    // west. (Being shoved back would show up as x above 8.95 + the overlap tolerance.)
    assertTrue(westMaxX < 9.0, "west-bound max x " + westMaxX);
    // And both keep cycling afterwards (30 s, 7 + 0.5 s dwell per leg at most, so >= 2 scores).
    assertTrue(w.opponents().get(0).behavior().cyclesCompleted() >= 2);
    assertTrue(w.opponents().get(1).behavior().cyclesCompleted() >= 2);
  }

  @Test
  void theLowerIdBacksOutWhicheverWayItIsGoing() {
    SimWorld w = headOn(1, 0); // now the west-bound robot has the lower id
    RobotBody east = body(w, 1);
    RobotBody west = body(w, 0);
    double westMaxXAfterStall = Double.NEGATIVE_INFINITY;
    double eastThroughAt = Double.NaN;
    for (int i = 0; i < (int) Math.round(15.0 / DT); i++) {
      w.step(ChassisCommand.STOP, DT);
      if (w.timeSec() > 3.0) {
        westMaxXAfterStall = Math.max(westMaxXAfterStall, west.position().x());
      }
      if (Double.isNaN(eastThroughAt) && east.position().x() > 10.6) {
        eastThroughAt = w.timeSec();
      }
    }
    assertTrue(eastThroughAt < 10.0, "east-bound robot through the gap at " + eastThroughAt);
    // mirror image: the west-bound robot backed out east past its yield point x = 8.8 + 1.5 = 10.3
    assertTrue(westMaxXAfterStall > 10.0, "west-bound max x " + westMaxXAfterStall);
  }

  @Test
  void aCyclerNeverBacksOutFromOurRobot() {
    // Our robot sits in the gap at (8.95, 4.035) and pushes west at full speed; one Cycler comes
    // east from (6, 4.035). They stall head-on in the gap (both full effort). It must keep
    // driving into us: that is what a Cycler models.
    FieldConfig f = gapField();
    RobotBody self = RobotBody.typical(100, new Pose(new Vec2(8.95, 4.035), Math.PI));
    RobotBody east = RobotBody.typical(0, new Pose(new Vec2(6.0, 4.035), 0.0));
    SimWorld w = new SimWorld(f, self, List.of(new SimWorld.Opponent(east,
        new CyclerBehavior(new Vec2(3.0, 4.035), new Vec2(13.0, 4.035), 0.3, 0.5))));
    ChassisCommand pushWest = new ChassisCommand(new Vec2(-4.5, 0.0), 0.0);
    int touchingLoops = 0;
    for (int i = 0; i < (int) Math.round(8.0 / DT); i++) {
      w.step(pushWest, DT);
      if (!w.selfTouchingNow().isEmpty()) {
        touchingLoops++;
        // still driving east into us, never backing out
        assertTrue(east.desiredVelocity().x() > 0.0, "t=" + w.timeSec() + " " + east.desiredVelocity());
      }
    }
    assertTrue(touchingLoops * DT > 4.0, "expected a long stall, touched for " + touchingLoops * DT);
  }

  @Test
  void twoLaneCyclersAt10msNoLongerDeadlock() {
    // The audit's failing input: two lanes, no defense, Cyclers, 180 s at dt = 10 ms. Before the
    // fix opponents #0 and #2 stopped in the lower gap at t ~ 27 s with 3 and 4 cycles. At 20 ms
    // (no meeting) each does about 24.
    FieldConfig f = FieldConfig.twoLanes();
    SimWorld w = Scenarios.three(f, Scenarios.OpponentKind.CYCLER, Scenarios.parkedOutOfTheWay(f));
    for (int i = 0; i < 18000; i++) {
      w.step(ChassisCommand.STOP, 0.01);
    }
    for (SimWorld.Opponent o : w.opponents()) {
      assertTrue(o.behavior().cyclesCompleted() >= 20,
          "#" + o.body().id() + " cycles " + o.behavior().cyclesCompleted());
    }
  }
}
