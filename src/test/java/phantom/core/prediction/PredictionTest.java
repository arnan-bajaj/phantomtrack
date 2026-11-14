package phantom.core.prediction;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.CutoffMode;
import phantom.core.modes.DefenseContext;
import phantom.core.tracking.TrackedRobot;

/** IntentPredictor and CutoffMode. */
class PredictionTest {
  private static final double TOL = 1e-9;
  /** 20 x 20 open field, goal (16,10), one pickup (2,10). */
  private static final FieldConfig LINE =
      new FieldConfig(20, 20, new Vec2(16, 10), List.of(new Vec2(2, 10)), 0.45);

  private static TrackedRobot opp(double x, double y, double vx, double vy) {
    return new TrackedRobot(0, new Vec2(x, y), new Vec2(vx, vy), 0.0, 1.0);
  }

  // ---------------- destination ----------------

  @Test
  void movingOpponentHeadsWhereItsVelocityPoints() {
    // at (10,10) moving (2,0): goal score (1,0).(1,0) = 1, pickup score (1,0).(-1,0) = -1 -> goal
    assertEquals(new Vec2(16, 10), IntentPredictor.destination(opp(10, 10, 2, 0), 0, LINE));
    // moving (-2,0): goal -1, pickup +1 -> pickup
    assertEquals(new Vec2(2, 10), IntentPredictor.destination(opp(10, 10, -2, 0), 0, LINE));
  }

  @Test
  void stoppedOpponentGuessesFromWhereItIs() {
    // (2.5,10) stopped: pickup 0.5 m away < goal 13.5 m -> just loaded, going to the goal
    assertEquals(new Vec2(16, 10), IntentPredictor.destination(opp(2.5, 10, 0, 0), 0, LINE));
    // (15,10) stopped: pickup 13 m > goal 1 m -> just scored, going back to the pickup
    assertEquals(new Vec2(2, 10), IntentPredictor.destination(opp(15, 10, 0, 0), 0, LINE));
  }

  // ---------------- route geometry ----------------

  @Test
  void straightRouteWhenClear() {
    var r = IntentPredictor.route(new Vec2(5, 10), new Vec2(16, 10), LINE);
    assertEquals(2, r.size());
    assertEquals(11.0, IntentPredictor.length(r), TOL);
    assertEquals(new Vec2(8, 10), IntentPredictor.pointAlong(r, 3.0));
  }

  @Test
  void routeGoesThroughTheGateWhenAWallBlocks() {
    // choke: (5,1) -> goal (13,4.035) hits the lower wall (at x = 8.15, y = 2.195 < 3.25)
    // -> [(5,1), gate (8.8,4.035), (13,4.035)]
    // leg 1 = hypot(3.8, 3.035) = 4.863252, leg 2 = 4.2, total = 9.063252
    var r = IntentPredictor.route(new Vec2(5, 1), FieldConfig.choke().defendedGoal(), FieldConfig.choke());
    assertEquals(3, r.size());
    assertEquals(new Vec2(8.8, 4.035), r.get(1));
    assertEquals(Math.hypot(3.8, 3.035) + 4.2, IntentPredictor.length(r), TOL);
    // 1.0 m past the gate along leg 2: (9.8, 4.035)
    Vec2 q = IntentPredictor.pointAlong(r, Math.hypot(3.8, 3.035) + 1.0);
    assertEquals(9.8, q.x(), TOL);
    assertEquals(4.035, q.y(), TOL);
  }

  @Test
  void pointAlongClampsToTheEnds() {
    var r = List.of(new Vec2(0, 0), new Vec2(3, 4));
    assertEquals(new Vec2(0, 0), IntentPredictor.pointAlong(r, -1));
    assertEquals(new Vec2(3, 4), IntentPredictor.pointAlong(r, 99));
  }

  // ---------------- CutoffMode ----------------

  private static DefenseContext ctx(Vec2 self, TrackedRobot o) {
    return new DefenseContext(0, new Pose(self, 0), Vec2.ZERO, List.of(o), LINE, -1);
  }

  @Test
  void cutoffGoesToTheFirstPointWeReachFirst() {
    // opp (6,10) moving (2,0) to goal (16,10); us at (12,14), speed 4.5, margin 0.3, s = 2
    // need |self - (6+a,10)| / 4.5 + 0.3 <= a / 2, a = 1.1, 1.35, ...
    //   a = 2.85: dist = hypot(2.85-6, 4) = hypot(-3.15, 4) = 5.0915 -> 1.4314 > 1.425  no
    //   a = 3.10: dist = hypot(-2.90, 4) = 4.9406 -> 1.3979 <= 1.55                     yes
    // -> target (9.1, 10); facing the opponent at (6,10): heading pi
    var t = CutoffMode.defaults().compute(ctx(new Vec2(12, 14), opp(6, 10, 2, 0))).orElseThrow();
    assertEquals(9.1, t.position().x(), 1e-9);
    assertEquals(10.0, t.position().y(), TOL);
    assertEquals(Math.PI, t.heading(), TOL);
    assertEquals(Vec2.ZERO, t.velocityFeedforward());
  }

  @Test
  void cutoffFallsBackToInFrontOfTheGoalWhenTooFar() {
    // us at (0,0), very slow (0.5 m/s): never first -> R(L - 0.5) = (16 - 0.5, 10) = (15.5, 10)
    var mode = new CutoffMode(1.1, 0.5, 0.3);
    var t = mode.compute(ctx(new Vec2(0, 0), opp(6, 10, 2, 0))).orElseThrow();
    assertEquals(15.5, t.position().x(), TOL);
    assertEquals(10.0, t.position().y(), TOL);
  }

  @Test
  void cutoffAmbushesTheNextRunWhenTheyHeadBack() {
    // opp (10,10) moving (-2,0) -> going to pickup (2,10). Next run (2,10) -> (16,10), length 14,
    // wait AMBUSH = 3 m before the goal: R'(14 - 3) = (13, 10); facing them at (10,10): heading pi
    var t = CutoffMode.defaults().compute(ctx(new Vec2(14, 12), opp(10, 10, -2, 0))).orElseThrow();
    assertEquals(13.0, t.position().x(), TOL);
    assertEquals(10.0, t.position().y(), TOL);
    assertEquals(Math.PI, t.heading(), TOL);
  }

  @Test
  void cutoffNoOpponentsAndBadParameters() {
    var empty = new DefenseContext(0, new Pose(5, 5, 0), Vec2.ZERO, List.of(), LINE, -1);
    assertTrue(CutoffMode.defaults().compute(empty).isEmpty());
    assertThrows(IllegalArgumentException.class, () -> new CutoffMode(0, 4.5, 0.3));
  }
}
