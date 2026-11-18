package phantom.core.manager;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.control.DriverBlend;
import phantom.core.control.HolonomicController;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.ShadowMode;
import phantom.core.tracking.TrackedRobot;

/** DefenseAssist as a whole: what reaches the command. */
class DefenseAssistTest {
  private static final double TOL = 1e-9;

  @Test
  void aNonFiniteTrackNeverReachesTheCommand() {
    // Audit H1, defense in depth: whatever a tracker reports, a track with a NaN or infinite
    // position, velocity or time is not given to the modes or the pin guard. Here the only track
    // is broken, so the assist sees nobody and searches: home = goal + 2.5 * unit(center - goal)
    // = (13, 4.035) + 2.5 * (-1, 0) = (10.5, 4.035). From (10.5, 4.035) facing pi at t = 0 the
    // search heading is pi + 0.6 * sin(0) = pi: zero error, so the command is exactly STOP.
    var broken =
        List.of(
            new TrackedRobot(0, new Vec2(Double.NaN, 4), Vec2.ZERO, 0.0, 1.0),
            new TrackedRobot(1, new Vec2(6, 4), new Vec2(Double.POSITIVE_INFINITY, 0), 0.0, 1.0),
            new TrackedRobot(2, new Vec2(6, 4), Vec2.ZERO, Double.NaN, 1.0));
    var assist =
        new DefenseAssist(
            List.of(ShadowMode.defaults()),
            HolonomicController.defaults(),
            DriverBlend.defaults(),
            now -> broken,
            FieldConfig.placeholder());
    ChassisCommand c =
        assist.update(0.0, new Pose(10.5, 4.035, Math.PI), Vec2.ZERO, ChassisCommand.STOP, true);
    assertTrue(c.velocity().isFinite() && Double.isFinite(c.omega()), c.toString());
    assertEquals(DefenseAssist.State.SEARCHING, assist.lastState());
    assertEquals(0.0, c.velocity().norm(), TOL);
    assertEquals(0.0, c.omega(), TOL);
    assertTrue(assist.lastOpponents().isEmpty());
  }

  @Test
  void theSearchHeadingIsWrapped() {
    // Audit L4. heading = angle(d) + 0.6 sin(0.8 t) with d = (-1, 0) on the placeholder field:
    // at t = pi / 1.6, sin(0.8 t) = sin(pi / 2) = 1 -> pi + 0.6 = 3.7415927 rad, outside
    // (-pi, pi] (CLAUDE.md rule 6). Wrapped: pi + 0.6 - 2 pi = -2.5415927.
    var assist =
        new DefenseAssist(
            List.of(ShadowMode.defaults()),
            HolonomicController.defaults(),
            DriverBlend.defaults(),
            now -> List.of(),
            FieldConfig.placeholder());
    assertEquals(0.6 - Math.PI, assist.searchTarget(Math.PI / 1.6).heading(), 1e-12);
    // at t = 0: pi + 0 = pi, already in range
    assertEquals(Math.PI, assist.searchTarget(0.0).heading(), 1e-12);
  }
}
