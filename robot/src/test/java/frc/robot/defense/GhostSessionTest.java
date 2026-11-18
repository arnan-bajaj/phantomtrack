package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.sim.Scenarios;

/** Ghosts on the robot: pin timer, rumble threshold, referee, and the controller gain. */
class GhostSessionTest {
  private static final double TOL = 1e-9;
  private static final double DT = 0.02;
  // Ghost #0 starts at pickup (1.5, 1.2) and drives straight at the goal (13.0, 4.035):
  // dir = (11.5, 2.835) / sqrt(11.5^2 + 2.835^2) = (11.5, 2.835) / 11.8442908
  private static final Vec2 PICKUP = new Vec2(1.5, 1.2);
  private static final Vec2 DIR = new Vec2(11.5, 2.835).times(1.0 / 11.8442908);
  // Our robot parked head-on in its path, touching: 0.9 m ahead of it.
  private static final Pose BLOCKING = new Pose(PICKUP.plus(DIR.times(0.9)), Math.PI);

  private static GhostSession session() {
    return new GhostSession(GhostSettings.firstTest(), SafetyConfig.firstTest(), BLOCKING);
  }

  @Test
  void pinTimerAndRumbleWarningWhileAGhostPushesIntoUs() {
    GhostSession s = session();
    // Touching from the first step; the episode opens at 0 s and adds 0.02 s per step after:
    // after N steps the timer is (N - 1) * 0.02.  N = 70 -> 1.38 s < 1.5;  N = 80 -> 1.58 s.
    for (int i = 0; i < 70; i++) {
      s.step(BLOCKING, Vec2.ZERO, Vec2.ZERO, DT);
    }
    assertEquals(1.38, s.pinSeconds(0), 1e-6);
    assertFalse(s.pinWarning(1.5));
    for (int i = 0; i < 10; i++) {
      s.step(BLOCKING, Vec2.ZERO, Vec2.ZERO, DT);
    }
    assertEquals(1.58, s.maxPinSeconds(), 1e-6);
    assertTrue(s.pinWarning(1.5));
    // The real robot was never moved and the ghost is held at the bumper (0.9 m).
    assertEquals(0.9, s.ghostPoses().get(0).position().distanceTo(BLOCKING.position()), 1e-6);
    // We were not pressing (desired velocity 0), so a referee calls nothing.
    assertEquals(0, s.world().pinViolations());
  }

  @Test
  void refereeCallsAPinWhenWePressForThreeSeconds() {
    // Pressing straight into it at the cap: effort = 1.0 / 1.0 = 1 > 0.1.
    // (N - 1) * 0.02 >= 3.0 at N = 151; run 155.
    GhostSession s = session();
    Vec2 press = DIR.times(-1.0);
    for (int i = 0; i < 155; i++) {
      s.step(BLOCKING, Vec2.ZERO, press, DT);
    }
    assertEquals(1, s.world().pinViolations());
  }

  @Test
  void ghostCountAndSpeedFollowTheSettings() {
    var three =
        new GhostSession(
            new GhostSettings(Scenarios.Layout.TWO_LANES, Scenarios.OpponentKind.EVADER, 3, 2.0),
            SafetyConfig.firstTest(),
            new Pose(10.0, 4.0, 0.0));
    assertEquals(3, three.ghostCount());
    assertEquals("Two lanes", three.field().name());
    assertEquals(2.0, three.contactGhosts().get(0).maxSpeed(), TOL);
    // Count is clamped to [0, 3].
    assertEquals(3, new GhostSettings(Scenarios.Layout.OPEN, Scenarios.OpponentKind.CYCLER, 9, 1.0).count());
    assertEquals(0, new GhostSettings(Scenarios.Layout.OPEN, Scenarios.OpponentKind.CYCLER, -2, 1.0).count());
  }

  @Test
  void positionGainCannotOvershootUnderTheAccelLimit() {
    // kP = min(3.0, 2 a / cap). a = 2, cap = 1: min(3, 4) = 3.  a = 2, cap = 4.5: 4 / 4.5 = 0.8889.
    assertEquals(3.0, GhostSession.positionGain(2.0, 1.0), TOL);
    assertEquals(4.0 / 4.5, GhostSession.positionGain(2.0, 4.5), TOL);
  }
}
