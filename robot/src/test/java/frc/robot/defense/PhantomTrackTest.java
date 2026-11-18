package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.manager.DefenseAssist;
import phantom.sim.Scenarios;

/**
 * The whole robot pipeline, without WPILib: every safety rule seen from the outside, plus a
 * closed-loop run where a perfect drivetrain follows the output and the assist has to track a
 * ghost. First-test settings: cap 1.0 m/s, accel 2.0 m/s^2, area [7.5, 12.5] x [1.5, 6.5],
 * margin 0.75, one 1.0 m/s Cycler on the open field.
 */
class PhantomTrackTest {
  private static final double TOL = 1e-9;
  private static final double DT = 0.02;
  private static final Pose START = new Pose(10.0, 4.035, Math.PI);

  private final PhantomTrack pt =
      new PhantomTrack(SafetyConfig.firstTest(), GhostSettings.firstTest());
  private int k = 0;

  private double now() {
    return DT * k;
  }

  /** One loop with a parked robot at pose, fresh timestamp. */
  private ChassisCommand loop(
      RobotMode mode, Pose pose, ChassisCommand driver, double deflection, double trigger) {
    ChassisCommand out =
        pt.update(
            new PhantomTrack.Input(
                now(), mode, pose, Vec2.ZERO, now(), driver, deflection, trigger));
    k++;
    return out;
  }

  /** Teleop, sticks centered, trigger released, for n loops: lets the pose monitor turn healthy. */
  private void settle(int n) {
    for (int i = 0; i < n; i++) {
      loop(RobotMode.TELEOP, START, ChassisCommand.STOP, 0.0, 0.0);
    }
  }

  @Test
  void autonomousAndDisabledCommandStopAndFreezeTheGhosts() {
    ChassisCommand driver = new ChassisCommand(new Vec2(1.0, 0.0), 0.5);
    for (RobotMode m : new RobotMode[] {RobotMode.AUTONOMOUS, RobotMode.DISABLED, RobotMode.TEST}) {
      for (int i = 0; i < 50; i++) {
        assertSame(ChassisCommand.STOP, loop(m, START, driver, 1.0, 1.0));
      }
      assertEquals(SafetyGate.Decision.NOT_TELEOP, pt.decision());
    }
    assertEquals(0.0, pt.ghostTimeSec(), 0.0); // ghosts never moved
  }

  @Test
  void withoutTheDeadmanTheDriverDrivesUntouched() {
    settle(40);
    // 3.0 m/s is over the assist cap; the driver's command is not limited.
    ChassisCommand driver = new ChassisCommand(new Vec2(3.0, -1.0), 1.0);
    assertSame(driver, loop(RobotMode.TELEOP, START, driver, 0.9, 0.0));
    assertEquals(SafetyGate.Decision.OFF, pt.decision());
  }

  @Test
  void engagedOutputIsCappedAndAccelLimited() {
    settle(40);
    // The pose must follow the command: a pose that never changes while we command motion is
    // (correctly) flagged FROZEN after 0.5 s. Perfect drivetrain: pose += v * dt.
    Pose pose = START;
    ChassisCommand prev = ChassisCommand.STOP;
    for (int i = 0; i < 50; i++) {
      ChassisCommand out = loop(RobotMode.TELEOP, pose, ChassisCommand.STOP, 0.0, 1.0);
      assertEquals(SafetyGate.Decision.ASSIST, pt.decision(), "loop " + i);
      assertTrue(out.velocity().norm() <= 1.0 + TOL, "over the cap: " + out);
      // 2.0 m/s^2 * 0.02 s = 0.04 m/s per loop at most. In 1 s from rest we cover at most
      // 0.5 * 2 * 0.5^2 + 1.0 * 0.5 = 0.75 m, so from (10, 4.035) every fence line stays more
      // than 1 m away: sqrt(2 * 2 * 1.0) = 2 m/s > cap, the fence never cuts in this test.
      assertTrue(out.velocity().minus(prev.velocity()).norm() <= 0.04 + TOL, "too sudden: " + out);
      assertTrue(Math.abs(out.omega()) <= 2.0 + TOL);
      prev = out;
      pose =
          new Pose(
              pose.position().plus(out.velocity().times(DT)),
              Angles.wrap(pose.heading() + out.omega() * DT));
    }
    assertTrue(pt.target().isPresent());
  }

  @Test
  void aPoseThatDoesNotMoveWhileWeDriveIsFrozen() {
    // Same as above but the pose is stuck (e.g. the estimator thread died): after the command
    // passes 0.2 m/s (5 loops at 0.04) and 0.5 s more, the monitor reports FROZEN and the
    // assist lets go. 60 loops = 1.2 s is plenty.
    settle(40);
    for (int i = 0; i < 60; i++) {
      loop(RobotMode.TELEOP, START, ChassisCommand.STOP, 0.0, 1.0);
    }
    assertEquals(SafetyGate.Decision.POSE_FAULT, pt.decision());
    assertTrue(pt.status().contains("FROZEN"), pt.status());
  }

  @Test
  void anyStickMovementHandsControlBack() {
    settle(40);
    loop(RobotMode.TELEOP, START, ChassisCommand.STOP, 0.0, 1.0);
    ChassisCommand driver = new ChassisCommand(new Vec2(0.2, 0.0), 0.0);
    assertSame(driver, loop(RobotMode.TELEOP, START, driver, 0.15, 1.0));
    assertEquals(SafetyGate.Decision.DRIVER_OVERRIDE, pt.decision());
  }

  @Test
  void poseJumpTurnsTheAssistOffUntilTheTriggerIsReleased() {
    settle(40);
    loop(RobotMode.TELEOP, START, ChassisCommand.STOP, 0.0, 1.0);
    assertEquals(SafetyGate.Decision.ASSIST, pt.decision());
    // 0.6 m jump in one loop.
    Pose jumped = new Pose(10.6, 4.035, Math.PI);
    ChassisCommand out = loop(RobotMode.TELEOP, jumped, ChassisCommand.STOP, 0.0, 1.0);
    assertEquals(SafetyGate.Decision.POSE_FAULT, pt.decision());
    assertSame(ChassisCommand.STOP, out); // the driver's (centered) command, not the assist's
    assertEquals(PoseMonitor.Fault.POSITION_JUMP, pt.currentFault());
    // 2 s of a good pose with the trigger still held: still off.
    for (int i = 0; i < 100; i++) {
      loop(RobotMode.TELEOP, jumped, ChassisCommand.STOP, 0.0, 1.0);
    }
    assertTrue(pt.poseHealthy());
    assertEquals(SafetyGate.Decision.POSE_FAULT, pt.decision());
    loop(RobotMode.TELEOP, jumped, ChassisCommand.STOP, 0.0, 0.0); // release
    loop(RobotMode.TELEOP, jumped, ChassisCommand.STOP, 0.0, 1.0); // press again
    assertEquals(SafetyGate.Decision.ASSIST, pt.decision());
  }

  @Test
  void stalePoseTurnsTheAssistOff() {
    settle(40);
    ChassisCommand out =
        pt.update(
            new PhantomTrack.Input(
                now(), RobotMode.TELEOP, START, Vec2.ZERO, now() - 0.3, ChassisCommand.STOP, 0.0,
                1.0));
    assertEquals(SafetyGate.Decision.POSE_FAULT, pt.decision());
    assertSame(ChassisCommand.STOP, out);
  }

  @Test
  void nanPoseNeverReachesTheCommand() {
    settle(40);
    Pose bad = new Pose(Double.NaN, 4.0, 0.0);
    ChassisCommand out = loop(RobotMode.TELEOP, bad, ChassisCommand.STOP, 0.0, 1.0);
    assertTrue(out.velocity().isFinite() && Double.isFinite(out.omega()));
    assertEquals(SafetyGate.Decision.POSE_FAULT, pt.decision());
  }

  @Test
  void outsideTheAreaTheAssistDoesNothing() {
    Pose outside = new Pose(5.0, 4.0, 0.0); // x = 5.0 < 7.5
    for (int i = 0; i < 40; i++) {
      loop(RobotMode.TELEOP, outside, ChassisCommand.STOP, 0.0, 0.0);
    }
    assertSame(ChassisCommand.STOP, loop(RobotMode.TELEOP, outside, ChassisCommand.STOP, 0.0, 1.0));
    assertEquals(SafetyGate.Decision.OUTSIDE_AREA, pt.decision());
  }

  @Test
  void settingsWaitForTheTriggerToBeReleased() {
    settle(40);
    loop(RobotMode.TELEOP, START, ChassisCommand.STOP, 0.0, 1.0);
    pt.requestSettings(
        SafetyConfig.firstTest(),
        new GhostSettings(Scenarios.Layout.CHOKE, Scenarios.OpponentKind.EVADER, 3, 1.5));
    loop(RobotMode.TELEOP, START, ChassisCommand.STOP, 0.0, 1.0);
    assertTrue(pt.settingsPending());
    assertEquals(1, pt.ghostPoses().size());
    loop(RobotMode.TELEOP, START, ChassisCommand.STOP, 0.0, 0.0); // released
    assertFalse(pt.settingsPending());
    assertEquals(3, pt.ghostPoses().size());
    assertEquals("Choke", pt.field().name());
  }

  @Test
  void focusCyclesThroughAutoAndEveryGhost() {
    settle(2); // builds the session: 1 ghost
    // n = 1: next = floorMod(focus + 2, 2) - 1: -1 -> 0 -> -1. Backwards from -1: floorMod(-1, 2)
    // - 1 = 0.
    pt.cycleFocus(+1);
    assertEquals(0, pt.focus());
    pt.cycleFocus(+1);
    assertEquals(-1, pt.focus());
    pt.cycleFocus(-1);
    assertEquals(0, pt.focus());
    // n = 3: -1 -> 0 -> 1 -> 2 -> -1; backwards from -1: floorMod(-1, 4) - 1 = 2.
    pt.cycleFocus(-1); // back to auto
    pt.requestSettings(
        SafetyConfig.firstTest(),
        new GhostSettings(Scenarios.Layout.OPEN, Scenarios.OpponentKind.CYCLER, 3, 1.0));
    settle(1);
    pt.cycleFocus(-1);
    assertEquals(2, pt.focus());
    pt.cycleFocus(+1);
    assertEquals(-1, pt.focus());
  }

  @Test
  void modeHoldAndNames() {
    settle(2);
    assertEquals("Line mirror", pt.modeName());
    pt.selectMode(GhostSession.MODE_CUTOFF);
    assertEquals("Cut-off", pt.modeName());
    pt.selectMode(GhostSession.MODE_SHADOW);
    pt.toggleHold();
    assertEquals("Shadow + hold", pt.modeName());
    pt.selectMode(GhostSession.MODE_ZONE);
    assertEquals("Zone + hold", pt.modeName());
    pt.selectMode(7); // ignored
    assertEquals(GhostSession.MODE_ZONE, pt.modeIndex());
  }

  @Test
  void closedLoopTheAssistTracksTheGhostAndStaysInsideTheFence() {
    // A perfect drivetrain: pose += command * dt. 60 s of Line mirror against one Cycler.
    Pose pose = START;
    settle(40);
    double errSum = 0.0;
    int errSamples = 0;
    double maxSpeed = 0.0;
    double lastBackoffSec = Double.NEGATIVE_INFINITY;
    Geofence fence = pt.fence();
    Vec2 velocity = Vec2.ZERO; // a perfect drivetrain moves at exactly the last command
    for (int i = 0; i < 3000; i++) {
      ChassisCommand out =
          pt.update(
              new PhantomTrack.Input(
                  now(), RobotMode.TELEOP, pose, velocity, now(), ChassisCommand.STOP, 0.0, 1.0));
      velocity = out.velocity();
      k++;
      assertEquals(SafetyGate.Decision.ASSIST, pt.decision(), "loop " + i);
      maxSpeed = Math.max(maxSpeed, out.velocity().norm());
      pose =
          new Pose(
              pose.position().plus(out.velocity().times(DT)),
              Angles.wrap(pose.heading() + out.omega() * DT));
      // The fence line, plus the 0.4 mm discrete-step allowance derived in GeofenceTest.
      Vec2 p = pose.position();
      assertTrue(p.x() >= fence.minX() + fence.margin() - 0.0005, "west " + p);
      assertTrue(p.x() <= fence.maxX() - fence.margin() + 0.0005, "east " + p);
      assertTrue(p.y() >= fence.minY() + fence.margin() - 0.0005, "south " + p);
      assertTrue(p.y() <= fence.maxY() - fence.margin() + 0.0005, "north " + p);
      if (pt.coreState() == DefenseAssist.State.BACKING_OFF) {
        lastBackoffSec = now();
      }
      // After the first 10 s, while the mode (not the pin guard) is in charge. A back-off steps
      // the robot up to ~2 m away (EscapePlanner, to the fence at worst); getting back at the
      // 1.0 m/s cap takes up to ~3 s, so the first 3 s after one are recovery, not tracking.
      if (i >= 500
          && pt.coreState() == DefenseAssist.State.ENGAGED
          && now() - lastBackoffSec >= 3.0
          && pt.target().isPresent()) {
        // Where the mode wants us, limited to the fence (we can't be asked to go past it).
        Vec2 t = pt.target().get().position();
        Vec2 reachable =
            new Vec2(
                Math.max(fence.minX() + fence.margin(), Math.min(fence.maxX() - fence.margin(), t.x())),
                Math.max(fence.minY() + fence.margin(), Math.min(fence.maxY() - fence.margin(), t.y())));
        errSum += p.distanceTo(reachable);
        errSamples++;
      }
    }
    assertTrue(maxSpeed <= 1.0 + TOL);
    assertTrue(maxSpeed > 0.5, "the assist barely moved: " + maxSpeed);
    assertEquals(0, pt.guardedId()); // the one ghost
    double meanErr = errSum / errSamples;
    assertTrue(meanErr < 0.25, "mean distance from the (reachable) target: " + meanErr);
  }

  @Test
  void pinGuardMarginComesFromTheTestCaps() {
    // RulesGuard.safetyMarginFor at 1.0 m/s, 2.0 m/s^2, assumed latency 0.1 s:
    //   slide d = 2 * 0.45 + 0.05 = 0.95 m: d_acc = 1/4 = 0.25 -> T = 1/2 + (0.95 - 0.25)/1 = 1.2 s
    //   margin = 0.5 + 0.1 + 0.02 + 1.2 = 1.82 s -> backs off at 3.0 - 1.82 = 1.18 s of contact
    GhostSession s = new GhostSession(GhostSettings.firstTest(), SafetyConfig.firstTest(), START);
    assertEquals(1.82, s.assist().pinSafetyMargin(), 1e-9);
  }

  @Test
  void oneGhostContactStaysUnderTwoAndAHalfSecondsWithLatency() {
    // 300 s of Cut-off + hold against one 1.0 m/s Cycler, perfect drivetrain that applies each
    // command 0 or 100 ms late (PinMarginStudy). With the old margin (1.17 s, no latency) the
    // any-touch timer peaked at 3.00 s / 3.10 s; the target is 2.5 s (0.5 s of real slack).
    for (int latencyLoops : new int[] {0, 5}) {
      PinMarginStudy.Result r =
          PinMarginStudy.run(
              new PinMarginStudy.Config(
                  1.0, 2.0, latencyLoops, Scenarios.OpponentKind.CYCLER, 1,
                  GhostSession.MODE_CUTOFF),
              300.0,
              false);
      assertTrue(r.peak() <= 2.5 + 1e-6, latencyLoops * 20 + " ms: peak " + r.peak());
      assertTrue(r.peak() > 1.0, "expected real contact in this run: " + r.peak());
      assertEquals(0, r.refPins());
    }
  }
}
