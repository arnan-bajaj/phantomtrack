package phantom.sim;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleFunction;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.Measurement;
import phantom.core.tracking.TrackedRobot;

/** Identity-free scoring, the simulated driver, and the anonymous pipeline in the loop. */
class AnonymousTrackingSimTest {
  private static final double TOL = 1e-9;

  private static TrackedRobot track(int id, double x, double y, double stamp) {
    return new TrackedRobot(id, new Vec2(x, y), Vec2.ZERO, stamp, 1.0);
  }

  // ---------------- scoring ----------------

  @Test
  void matchingByHandWithAndWithoutMemory() {
    // tracks T10 (0,0), T11 (1.2,0), T12 (10,10); robots R0 (1,0), R1 (-0.2,0), R2 (5,5)
    //   |T10-R0| = 1.0, |T10-R1| = 0.2, |T11-R0| = 0.2, |T11-R1| = 1.4, T12 is > 1.5 m from all
    List<TrackedRobot> tracks =
        List.of(track(10, 0, 0, 0), track(11, 1.2, 0, 0), track(12, 10, 10, 0));
    List<Vec2> robots = List.of(new Vec2(1, 0), new Vec2(-0.2, 0), new Vec2(5, 5));
    // No memory: (T10-R0, T11-R1) = 1.0 + 1.4 = 2.4 vs (T10-R1, T11-R0) = 0.2 + 0.2 = 0.4 -> swap
    assertArrayEquals(new int[] {1, 0, -1}, SimSession.matchTracks(tracks, 0, robots, 1.5));
    // Last step R0 was on T10: still 1.0 <= 1.5 away, so the pair is kept (CLEAR-MOT step 1),
    // then T11-R1 = 1.4 <= 1.5 is the only pair left.
    assertArrayEquals(new int[] {0, 1, -1},
        SimSession.matchTracks(tracks, 0, robots, 1.5, new int[] {10, -1, -1}));
    // A remembered pair that is now too far apart is dropped: R0 was on T12 (7.07 m away)
    assertArrayEquals(new int[] {1, 0, -1},
        SimSession.matchTracks(tracks, 0, robots, 1.5, new int[] {12, -1, -1}));
  }

  @Test
  void matchingUsesThePositionExtrapolatedToNow() {
    // stamped at 0.5 at (0,0) moving (1,0): at now = 1.0 it is at (0,0) + (1,0) * 0.5 = (0.5, 0)
    var moving = new TrackedRobot(3, Vec2.ZERO, new Vec2(1, 0), 0.5, 1.0);
    assertArrayEquals(new int[] {1},
        SimSession.matchTracks(List.of(moving), 1.0, List.of(new Vec2(-1.1, 0), new Vec2(0.5, 0)),
            1.5));
  }

  /** Parked robots: opponents at (3,3), (6,3), (9,3), one partner at (12,6), us at (14,1). */
  private static SimWorld parkedWorld() {
    List<SimWorld.Opponent> opps = new ArrayList<>();
    double[] xs = {3, 6, 9};
    for (int i = 0; i < 3; i++) {
      Vec2 p = new Vec2(xs[i], 3);
      opps.add(new SimWorld.Opponent(RobotBody.typical(i, new Pose(p, 0)),
          new CyclerBehavior(p, p, 0.0, 1000.0)));
    }
    Vec2 q = new Vec2(12, 6);
    var partner = new SimWorld.Opponent(RobotBody.typical(200, new Pose(q, 0)),
        new CyclerBehavior(q, q, 0.0, 1000.0));
    return new SimWorld(FieldConfig.placeholder(), RobotBody.typical(100, new Pose(14, 1, 0)),
        opps, List.of(partner));
  }

  @Test
  void metricsByHandOnAScriptedTracker() {
    // 0 .. 1 s:  T7 exactly on opponent 0, T8 at (3,6) (3 m from everyone: false, fresh),
    //            T9 exactly on the partner (fresh)
    // 1 .. 2 s:  T20 on opponent 0 but 0.5 m off (0.3, 0.4), T8 still there but last seen at
    //            0.5 s (stale: age > 0.3 s), T9 unchanged
    DoubleFunction<List<TrackedRobot>> script = t -> t < 1.0 - 1e-9
        ? List.of(track(7, 3, 3, t), track(8, 3, 6, t), track(9, 12, 6, t))
        : List.of(track(20, 3.3, 3.4, t), track(8, 3, 6, 0.5), track(9, 12, 6, t));
    var s = new SimSession(parkedWorld(), SimSession.Sensor.REALISTIC,
        SimSession.Tracking.ANONYMOUS, script::apply);
    for (int i = 0; i < 100; i++) { // 2 s, assist off: nobody moves
      s.step(ChassisCommand.STOP, false);
    }
    SimSession.Metrics m = s.metrics();
    assertEquals(2.0, m.seconds(), 1e-9);
    // opponent 0 always has a track (T7, then T20); 1 and 2 never: mean = (1 + 0 + 0) / 3
    assertEquals(1.0, m.coverage(0), TOL);
    assertEquals(0.0, m.coverage(1), TOL);
    assertEquals(1.0 / 3.0, m.meanCoverage(), TOL);
    assertEquals(1, m.idSwitches()); // opponent 0: T7 -> T20
    // RMSE: 50 steps at 0 m and 50 at |(0.3, 0.4)| = 0.5 m: sqrt((50*0 + 50*0.25) / 100) = 0.3536
    assertEquals(Math.sqrt(0.125), m.positionRmse(), 1e-9);
    // T8: 100 steps x 0.02 s = 2.0 track-seconds on no robot; fresh only for the first 50 = 1.0
    assertEquals(2.0, m.falseTrackSeconds(), 1e-9);
    assertEquals(1.0, m.freshFalseTrackSeconds(), 1e-9);
    // T9: on the partner all 2 s, always fresh
    assertEquals(2.0, m.partnerAsOpponentSeconds(), 1e-9);
    assertEquals(2.0, m.freshPartnerAsOpponentSeconds(), 1e-9);
  }

  @Test
  void simulatedDriverRepointsHalfASecondAfterTheTrackChanges() {
    // Opponent 0's track is T7 until 1.0 s, then T20. Locked on opponent 0:
    //   step at t = 0.02: the driver points the assist at T7 (first designation, not counted)
    //   the step at t = 1.0 still sees T7; the record after it sees T20 on opponent 0
    //   t = 1.02: focus T7 != T20 -> the driver notices; 1.02 + 0.5 = 1.52: re-points to T20
    DoubleFunction<List<TrackedRobot>> script =
        t -> List.of(track(t < 1.0 - 1e-9 ? 7 : 20, 3, 3, t));
    var s = new SimSession(parkedWorld(), SimSession.Sensor.REALISTIC,
        SimSession.Tracking.ANONYMOUS, script::apply);
    s.lockOn(0);
    assertEquals(SimSession.NOT_DESIGNATED, s.assist().focusOpponentId());
    s.step(ChassisCommand.STOP, true); // t = 0: nothing matched yet
    s.step(ChassisCommand.STOP, true); // t = 0.02: designate T7
    assertEquals(7, s.assist().focusOpponentId());
    while (s.world().timeSec() < 1.48 - 1e-9) {
      s.step(ChassisCommand.STOP, true);
    }
    assertEquals(7, s.assist().focusOpponentId()); // still waiting for the driver
    while (s.world().timeSec() < 1.60 - 1e-9) {
      s.step(ChassisCommand.STOP, true);
    }
    assertEquals(20, s.assist().focusOpponentId());
    assertEquals(1, s.redesignations());
  }

  // ---------------- the real pipeline ----------------

  @Test
  void partnersOffAndKnownIdGiveExactlyTheOldSession() {
    var old = SimSession.standard(Scenarios.OpponentKind.EVADER, SimSession.Sensor.REALISTIC, 3);
    var now = SimSession.standard(Scenarios.OpponentKind.EVADER, SimSession.Sensor.REALISTIC, 3,
        Scenarios.Layout.OPEN, false, SimSession.Tracking.KNOWN_ID, SimSession.Variant.STANDARD);
    for (var s : List.of(old, now)) {
      s.assist().setModeIndex(3);
      s.assist().setHoldEnabled(true);
      s.lockOn(0);
    }
    for (int i = 0; i < 1000; i++) { // 20 s
      old.step(ChassisCommand.STOP, true);
      now.step(ChassisCommand.STOP, true);
      assertEquals(old.world().self().position(), now.world().self().position());
    }
  }

  @Test
  void knownIdViewKeepsOnlyOpponentsWithTheirIds() {
    // Opponent 0 at (8,4) and a partner at (8,6), both in view; lots of clutter.
    List<SimWorld.Opponent> opps = List.of(new SimWorld.Opponent(
        RobotBody.typical(0, new Pose(8, 4, 0)), new CyclerBehavior(new Vec2(8, 4),
            new Vec2(8, 4), 0.0, 1000.0)));
    List<SimWorld.Opponent> partners = List.of(new SimWorld.Opponent(
        RobotBody.typical(200, new Pose(8, 6, 0)), new CyclerBehavior(new Vec2(8, 6),
            new Vec2(8, 6), 0.0, 1000.0)));
    var w = new SimWorld(FieldConfig.placeholder(), RobotBody.typical(100, new Pose(5, 4, 0)),
        opps, partners);
    var cam = new NoisySensor.Config("test", 0.0, 0.0, 0.0, 70, 8.0, 50);
    var sensor = new AnonymousSensor(w, new AnonymousSensor.Config(cam, 0, 3.0, 0.5, 1.0, true), 1);
    List<Measurement> got = sensor.knownIdView().poll(0.0);
    assertEquals(1, got.size());
    assertEquals(0, got.get(0).id());
    assertEquals(new Vec2(8, 4), got.get(0).position());
  }

  @Test
  void noNaNReachesACommandWithTheAnonymousTrackerInTheLoop() {
    for (int mode = 0; mode < 5; mode++) {
      for (boolean locked : new boolean[] {false, true}) {
        var s = SimSession.standard(Scenarios.OpponentKind.EVADER, SimSession.Sensor.BAD, mode + 1,
            Scenarios.Layout.CHOKE, true, SimSession.Tracking.ANONYMOUS,
            SimSession.Variant.STANDARD);
        s.assist().setModeIndex(mode);
        s.assist().setHoldEnabled(true);
        if (locked) {
          s.lockOn(0);
        }
        for (int i = 0; i < 1500; i++) { // 30 s
          s.step(ChassisCommand.STOP, true);
          ChassisCommand c = s.lastCommand();
          assertTrue(c.velocity().isFinite() && Double.isFinite(c.omega()),
              "mode " + mode + " step " + i + ": " + c);
          assertTrue(s.world().self().position().isFinite());
        }
        assertTrue(s.multiTracker().framesProcessed() > 300);
      }
    }
  }
}
