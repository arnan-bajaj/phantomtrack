package phantom.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.control.RulesGuard;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.manager.DefenseAssist;
import phantom.core.modes.DefenseContext;
import phantom.core.tracking.Measurement;
import phantom.core.tracking.TrackedRobot;

/** Simulated camera, search behavior, focus lock and stall detection. */
class SensingTest {
  private static final double TOL = 1e-9;

  private static SimWorld world(Pose self, Vec2... opps) {
    List<SimWorld.Opponent> list = new ArrayList<>();
    int id = 0;
    for (Vec2 p : opps) {
      list.add(new SimWorld.Opponent(RobotBody.typical(id++, new Pose(p, 0)),
          new CyclerBehavior(p, p, 0.0, 1000.0))); // parked: already at its "goal"
    }
    return new SimWorld(FieldConfig.placeholder(), RobotBody.typical(100, self), list);
  }

  @Test
  void cameraSeesOnlyInsideFovAndRange() {
    // We face +x from (5,4). Opponent 0 at (8,4): 3 m ahead (seen). Opponent 1 at (2,4): behind.
    // Opponent 2 at (14,4): 9 m ahead, beyond 8 m range.
    var w = world(new Pose(5, 4, 0), new Vec2(8, 4), new Vec2(2, 4), new Vec2(14, 4));
    var cfg = new NoisySensor.Config("test", 0.0, 0.0, 0.0, 70, 8.0, 50);
    var sensor = new NoisySensor(w, cfg, 1);
    List<Measurement> got = sensor.poll(0.0);
    assertEquals(1, got.size());
    assertEquals(0, got.get(0).id());
    assertEquals(8.0, got.get(0).position().x(), TOL); // zero noise -> exact
  }

  @Test
  void cameraResultsArriveLate() {
    var w = world(new Pose(5, 4, 0), new Vec2(8, 4));
    var cfg = new NoisySensor.Config("test", 0.0, 0.1, 0.0, 70, 8.0, 50);
    var sensor = new NoisySensor(w, cfg, 1);
    assertTrue(sensor.poll(0.0).isEmpty()); // captured at 0.0, due at 0.1
    assertTrue(sensor.poll(0.06).isEmpty());
    List<Measurement> got = sensor.poll(0.1);
    assertEquals(1, got.size());
    assertEquals(0.0, got.get(0).timestampSec(), TOL); // stamped with capture time
  }

  @Test
  void dropoutIsSeededAndRoughlyTheRequestedRate() {
    var w = world(new Pose(5, 4, 0), new Vec2(8, 4));
    var cfg = new NoisySensor.Config("test", 0.0, 0.0, 0.3, 70, 8.0, 50);
    int seenA = 0;
    int seenB = 0;
    var a = new NoisySensor(w, cfg, 7);
    var b = new NoisySensor(w, cfg, 7);
    for (int i = 0; i < 1000; i++) {
      seenA += a.poll(i * 0.02).size();
      seenB += b.poll(i * 0.02).size();
    }
    assertEquals(seenA, seenB); // same seed, same result
    assertEquals(700, seenA, 50); // ~70% of 1000 frames kept
  }

  @Test
  void selectedFocusThatIsNotTrackedGivesNoTarget() {
    var field = FieldConfig.placeholder();
    var ctx = new DefenseContext(0, new Pose(5, 4, 0), Vec2.ZERO,
        List.of(new TrackedRobot(1, new Vec2(6, 4), Vec2.ZERO, 0, 1)), field, 0);
    assertTrue(ctx.focusOpponent().isEmpty()); // asked for #0, only #1 visible: search for #0
    var auto = new DefenseContext(0, new Pose(5, 4, 0), Vec2.ZERO,
        List.of(new TrackedRobot(1, new Vec2(6, 4), Vec2.ZERO, 0, 1)), field, -1);
    assertEquals(1, auto.focusOpponent().orElseThrow().id());
  }

  @Test
  void searchTargetIsHomeInFrontOfGoal() {
    // placeholder field: goal (13, 4.035), center (8.27, 4.035): d = (-4.73,0)/4.73 = (-1,0)
    // home = (13,4.035) + 2.5*(-1,0) = (10.5, 4.035); heading at t=0 = angle(-1,0) + 0.6*sin(0) = pi
    SimSession s = SimSession.standard();
    var t = s.assist().searchTarget(0.0);
    assertEquals(10.5, t.position().x(), TOL);
    assertEquals(4.035, t.position().y(), TOL);
    assertEquals(Math.PI, t.heading(), TOL);
    assertEquals(-1, t.opponentId());
  }

  @Test
  void assistSearchesWhenNobodyIsVisible() {
    // Realistic camera, robot starts facing away from everyone 8+ m away: nothing tracked yet
    SimSession s = SimSession.standard(Scenarios.OpponentKind.CYCLER, SimSession.Sensor.REALISTIC, 1);
    s.assist().setFocusOpponentId(0);
    s.step(ChassisCommand.STOP, true);
    assertEquals(DefenseAssist.State.SEARCHING, s.assist().lastState());
  }

  @Test
  void stallDetection() {
    // asked for (3,0), moving (0.5,0): 0.5 < 0.3*3 = 0.9 -> blocked
    assertTrue(RulesGuard.isBlocked(new Vec2(3, 0), new Vec2(0.5, 0)));
    // moving (1.0,0): 1.0 >= 0.9 -> not blocked
    assertFalse(RulesGuard.isBlocked(new Vec2(3, 0), new Vec2(1.0, 0)));
    // asked for only 0.8 m/s: too slow to judge -> not blocked
    assertFalse(RulesGuard.isBlocked(new Vec2(0.8, 0), Vec2.ZERO));
    // sideways motion doesn't count as progress: (0,2).(1,0) = 0 < 0.9 -> blocked
    assertTrue(RulesGuard.isBlocked(new Vec2(3, 0), new Vec2(0, 2)));
  }

  @Test
  void blindBackoffWhenBlockedByAnUntrackedRobot() {
    var field = new FieldConfig(20, 20, new Vec2(0, 4), List.of(), 0.45);
    var guard = RulesGuard.defaults(field); // trigger at 3.0 - 0.5 = 2.5 s
    Pose self = new Pose(5, 4, 0);
    Vec2 cmd = new Vec2(3, 0);
    var out = java.util.Optional.<phantom.core.modes.DefenseTarget>empty();
    for (int i = 0; i <= 130; i++) { // 2.6 s blocked, nobody tracked
      out = guard.apply(i * 0.02, self, List.of(), java.util.Optional.empty(), cmd, Vec2.ZERO);
    }
    assertTrue(guard.blindBackingOff(130 * 0.02));
    // reverse the push: p* = (5,4) - 1.0*(1,0) = (4,4), v_ff = -3*(1,0)
    assertEquals(4.0, out.orElseThrow().position().x(), TOL);
    assertEquals(-3.0, out.orElseThrow().velocityFeedforward().x(), TOL);
  }
}
