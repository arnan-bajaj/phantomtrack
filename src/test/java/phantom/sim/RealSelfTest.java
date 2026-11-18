package phantom.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/**
 * Phase 7: ghost opponents against a REAL robot ({@link SimWorld#stepWithRealSelf}). The real
 * robot's state is measured, so ghosts must never move it. Placeholder field: robotHalfSize =
 * 0.45, so two robots touch at 0.90 m center to center.
 */
class RealSelfTest {
  private static final double TOL = 1e-9;
  private static final double DT = 0.02;
  private static final int SELF_ID = 100;

  /** Our robot (typical body, max speed 4.5) and one ghost that always drives ghostCmd. */
  private static SimWorld world(Pose self, Pose ghost, ChassisCommand ghostCmd) {
    OpponentBehavior scripted =
        new OpponentBehavior() {
          public String name() {
            return "Scripted";
          }

          public ChassisCommand decide(double t, double dt, RobotBody me, SimWorld w) {
            return ghostCmd;
          }

          public int cyclesCompleted() {
            return 0;
          }

          public double averageCycleSeconds() {
            return Double.NaN;
          }
        };
    return new SimWorld(
        FieldConfig.placeholder(),
        RobotBody.typical(SELF_ID, self),
        List.of(new SimWorld.Opponent(RobotBody.typical(0, ghost), scripted)));
  }

  private static RobotBody ghost(SimWorld w) {
    return w.opponents().get(0).body();
  }

  @Test
  void ghostGivesUpAllOfTheOverlapAndTheRealRobotStaysPut() {
    // Both parked, 0.6 m apart: overlap = 0.9 - 0.6 = 0.3, n = (1,0).
    // Real self: ghost moves the whole 0.3 -> x = 8.6 + 0.3 = 8.9. Self stays at 8.0.
    var w = world(new Pose(8.0, 4.0, 0), new Pose(8.6, 4.0, 0), ChassisCommand.STOP);
    w.stepWithRealSelf(new Pose(8.0, 4.0, 0), Vec2.ZERO, Vec2.ZERO, DT);
    assertEquals(8.0, w.self().position().x(), 0.0); // exactly: never touched
    assertEquals(4.0, w.self().position().y(), 0.0);
    assertEquals(8.9, ghost(w).position().x(), TOL);
    assertEquals(4.0, ghost(w).position().y(), TOL);
    assertTrue(ghost(w).velocity().isFinite());
  }

  @Test
  void theNormalSimStillSharesTheOverlap() {
    // Same setup through step(): both idle -> net 0 -> share_B = 0.5, each gives up 0.15.
    // self x = 8.0 - 0.15 = 7.85, ghost x = 8.6 + 0.15 = 8.75. Proves step() is unchanged.
    var w = world(new Pose(8.0, 4.0, 0), new Pose(8.6, 4.0, 0), ChassisCommand.STOP);
    w.step(ChassisCommand.STOP, DT);
    assertEquals(7.85, w.self().position().x(), TOL);
    assertEquals(8.75, ghost(w).position().x(), TOL);
  }

  @Test
  void ghostDrivingIntoAParkedRealRobotStopsAtTheBumper() {
    // Ghost starts 1.5 m away driving -x at full speed for 2 s; alone it would go ~7 m.
    // It must end exactly 0.9 m from the real robot: x = 8.0 + 0.9 = 8.9.
    // Closing, so its speed along n = (1,0) becomes v_self . n = 0.
    var w = world(new Pose(8.0, 4.0, 0), new Pose(9.5, 4.0, Math.PI),
        new ChassisCommand(new Vec2(-4.5, 0), 0));
    for (int i = 0; i < 100; i++) {
      w.stepWithRealSelf(new Pose(8.0, 4.0, 0), Vec2.ZERO, Vec2.ZERO, DT);
    }
    assertEquals(8.0, w.self().position().x(), 0.0);
    assertEquals(8.9, ghost(w).position().x(), TOL);
    assertEquals(0.0, ghost(w).velocity().x(), TOL);
    assertTrue(w.selfContactSeconds() > 1.0);
  }

  @Test
  void realRobotDrivingIntoAParkedGhostCarriesItAlong() {
    // Real robot moves +x at 1.0 m/s from x = 7.0: after step k it is at 7.0 + 0.02 k.
    // Ghost parked at 8.0. They touch when self reaches 7.1; from then on the ghost sits exactly
    // 0.9 ahead and moves with us: after 50 steps self = 8.0, ghost = 8.9, ghost v_x = 1.0.
    var w = world(new Pose(7.0, 4.0, 0), new Pose(8.0, 4.0, 0), ChassisCommand.STOP);
    Vec2 v = new Vec2(1.0, 0.0);
    for (int k = 1; k <= 50; k++) {
      w.stepWithRealSelf(new Pose(7.0 + 0.02 * k, 4.0, 0), v, v, DT);
    }
    assertEquals(8.0, w.self().position().x(), TOL);
    assertEquals(8.9, ghost(w).position().x(), TOL);
    assertEquals(1.0, ghost(w).velocity().x(), TOL);
    assertEquals(0.0, ghost(w).velocity().y(), TOL);
  }

  @Test
  void realRobotBackingAwayDoesNotDragTheGhost() {
    // Touching (0.9 apart), ghost parked, real robot leaves at -1 m/s.
    // closing = (v_self - v_ghost) . n = (-1 - 0) * 1 = -1 < 0 -> ghost velocity untouched (0).
    var w = world(new Pose(8.0, 4.0, 0), new Pose(8.9, 4.0, 0), ChassisCommand.STOP);
    Vec2 v = new Vec2(-1.0, 0.0);
    for (int k = 1; k <= 10; k++) {
      w.stepWithRealSelf(new Pose(8.0 - 0.02 * k, 4.0, 0), v, v, DT);
    }
    assertEquals(8.9, ghost(w).position().x(), TOL);
    assertEquals(0.0, ghost(w).velocity().norm(), TOL);
  }

  @Test
  void refereeCountsAPinOnlyWhileTheRealRobotPressesIn() {
    // Touching from the start; ghost pushes -x at full speed, real robot parked at 8.0.
    // Pressing: desired = (4.5, 0), effort = 4.5 / 4.5 = 1 > 0.1 -> contact time counts.
    // The episode opens on step 1 (0 s) and adds 0.02 s per step after: (N - 1) * 0.02.
    //   N = 140 -> 2.78 s < 3.0 -> no pin yet.   N = 155 -> 3.08 s >= 3.0 -> exactly one pin.
    var pressing = world(new Pose(8.0, 4.0, 0), new Pose(8.9, 4.0, Math.PI),
        new ChassisCommand(new Vec2(-4.5, 0), 0));
    for (int i = 0; i < 140; i++) {
      pressing.stepWithRealSelf(new Pose(8.0, 4.0, 0), Vec2.ZERO, new Vec2(4.5, 0), DT);
    }
    assertEquals(0, pressing.pinViolations());
    for (int i = 0; i < 15; i++) {
      pressing.stepWithRealSelf(new Pose(8.0, 4.0, 0), Vec2.ZERO, new Vec2(4.5, 0), DT);
    }
    assertEquals(1, pressing.pinViolations());

    // Not pressing (desired = 0): effort 0 is not > 0.1, so being pushed on is never a pin.
    var passive = world(new Pose(8.0, 4.0, 0), new Pose(8.9, 4.0, Math.PI),
        new ChassisCommand(new Vec2(-4.5, 0), 0));
    for (int i = 0; i < 155; i++) {
      passive.stepWithRealSelf(new Pose(8.0, 4.0, 0), Vec2.ZERO, Vec2.ZERO, DT);
    }
    assertEquals(0, passive.pinViolations());
  }

  @Test
  void ghostsSeeTheRealPoseInTheirDelayedView() {
    // The history that Evaders react to must hold the measured pose, not a simulated one.
    var w = world(new Pose(8.0, 4.0, 0), new Pose(12.0, 4.0, 0), ChassisCommand.STOP);
    w.stepWithRealSelf(new Pose(3.0, 2.5, 0), Vec2.ZERO, Vec2.ZERO, DT);
    Vec2 seen = w.positionsAt(w.timeSec()).get(SELF_ID);
    assertEquals(3.0, seen.x(), 0.0);
    assertEquals(2.5, seen.y(), 0.0);
    assertEquals(0.02, w.timeSec(), TOL);
  }

  @Test
  void headingIsWrappedAndARealPoseOutsideTheFieldIsNotClamped() {
    // 3 pi wraps to pi. A (bad) pose 1 m behind the wall stays where the robot says it is:
    // clamping it would hide a localization problem from the safety checks.
    var w = world(new Pose(8.0, 4.0, 0), new Pose(12.0, 4.0, 0), ChassisCommand.STOP);
    w.stepWithRealSelf(new Pose(-1.0, 4.0, 3.0 * Math.PI), Vec2.ZERO, Vec2.ZERO, DT);
    assertEquals(Math.PI, w.self().heading(), TOL);
    assertEquals(-1.0, w.self().position().x(), 0.0);
  }

  @Test
  void aNormalStepAfterARealStepIsSimulatedAgain() {
    // Real step parks self at (5, 4) with zero velocity. Then a normal step at full command:
    // v = 0 + 6 * 0.02 = 0.12 m/s, x = 5 + 0.12 * 0.02 = 5.0024.
    var w = world(new Pose(8.0, 4.0, 0), new Pose(12.0, 4.0, 0), ChassisCommand.STOP);
    w.stepWithRealSelf(new Pose(5.0, 4.0, 0), Vec2.ZERO, Vec2.ZERO, DT);
    w.step(new ChassisCommand(new Vec2(4.5, 0), 0), DT);
    assertEquals(5.0024, w.self().position().x(), TOL);
  }
}
