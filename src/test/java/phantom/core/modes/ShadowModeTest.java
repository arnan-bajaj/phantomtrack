package phantom.core.modes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

class ShadowModeTest {
  private static final double TOL = 1e-9;

  /** 20 m x 20 m field so nothing gets clamped by the walls unless a test wants it. */
  private static FieldConfig field(Vec2 goal) {
    return new FieldConfig(20, 20, goal, List.of(), 0.45);
  }

  private static DefenseContext ctx(FieldConfig f, Vec2 self, TrackedRobot... opps) {
    return new DefenseContext(0.0, new Pose(self, 0.0), Vec2.ZERO, List.of(opps), f, -1);
  }

  private static TrackedRobot opp(int id, double x, double y, double vx, double vy) {
    return new TrackedRobot(id, new Vec2(x, y), new Vec2(vx, vy), 0.0, 1.0);
  }

  @Test
  void straightLineCase() {
    // Opponent (2,4), goal (8,4), d = 1: u = (6,0)/6 = (1,0), target = (2,4) + 1*(1,0) = (3,4).
    var mode = new ShadowMode(1.0, 0.0);
    var t = mode.compute(ctx(field(new Vec2(8, 4)), new Vec2(5, 5), opp(0, 2, 4, 0, 0))).orElseThrow();
    assertEquals(3.0, t.position().x(), TOL);
    assertEquals(4.0, t.position().y(), TOL);
    // Facing the opponent, who is in the -x direction: heading = pi.
    assertEquals(Math.PI, t.heading(), TOL);
    assertEquals(0, t.opponentId());
  }

  @Test
  void diagonalCase() {
    // Opponent (5,5), goal (15,15), d = sqrt(2): u = (1,1)/sqrt(2), target = (5,5) + (1,1) = (6,6).
    var mode = new ShadowMode(Math.sqrt(2), 0.0);
    var t = mode.compute(ctx(field(new Vec2(15, 15)), new Vec2(1, 1), opp(0, 5, 5, 0, 0))).orElseThrow();
    assertEquals(6.0, t.position().x(), TOL);
    assertEquals(6.0, t.position().y(), TOL);
    assertEquals(Math.toRadians(-135), t.heading(), TOL);
  }

  @Test
  void leadPredictsOpponentMotion() {
    // Opponent (2,4) moving (1,0) m/s, lead 0.5 s: predicted (2.5,4), target (3.5,4).
    var mode = new ShadowMode(1.0, 0.5);
    var t = mode.compute(ctx(field(new Vec2(8, 4)), new Vec2(5, 5), opp(0, 2, 4, 1, 0))).orElseThrow();
    assertEquals(3.5, t.position().x(), TOL);
    assertEquals(4.0, t.position().y(), TOL);
    // Feedforward = opponent velocity.
    assertEquals(new Vec2(1, 0), t.velocityFeedforward());
  }

  @Test
  void neverPlacesTargetPastTheGoal() {
    // Opponent 0.4 m from goal, standoff 1.0: target must be the goal itself, not 0.6 m beyond it.
    var mode = new ShadowMode(1.0, 0.0);
    var t = mode.compute(ctx(field(new Vec2(8, 4)), new Vec2(9, 4), opp(0, 7.6, 4, 0, 0))).orElseThrow();
    assertEquals(8.0, t.position().x(), TOL);
    assertEquals(4.0, t.position().y(), TOL);
  }

  @Test
  void opponentOnTheGoalGivesFiniteTarget() {
    var mode = new ShadowMode(1.0, 0.0);
    var t = mode.compute(ctx(field(new Vec2(8, 4)), new Vec2(9, 4), opp(0, 8, 4, 0, 0))).orElseThrow();
    assertTrue(t.position().isFinite());
    assertTrue(Double.isFinite(t.heading()));
  }

  @Test
  void noOpponentsMeansNoTarget() {
    var mode = new ShadowMode(1.0, 0.0);
    Optional<DefenseTarget> t = mode.compute(ctx(field(new Vec2(8, 4)), new Vec2(5, 5)));
    assertTrue(t.isEmpty());
  }

  @Test
  void picksNearestOpponentWhenNoFocus() {
    var mode = new ShadowMode(1.0, 0.0);
    var c = ctx(field(new Vec2(8, 4)), new Vec2(10, 10), opp(0, 2, 2, 0, 0), opp(1, 9, 9, 0, 0));
    assertEquals(1, mode.compute(c).orElseThrow().opponentId());
  }

  @Test
  void respectsDriverFocus() {
    var mode = new ShadowMode(1.0, 0.0);
    var f = field(new Vec2(8, 4));
    var c =
        new DefenseContext(
            0.0, new Pose(new Vec2(10, 10), 0), Vec2.ZERO,
            List.of(opp(0, 2, 2, 0, 0), opp(1, 9, 9, 0, 0)), f, 0);
    assertEquals(0, mode.compute(c).orElseThrow().opponentId());
  }

  @Test
  void targetStaysInsideField() {
    // Goal near the wall at y = 0.2; target would land at y < half-size, so it gets clamped.
    var f = new FieldConfig(20, 20, new Vec2(10, 0.2), List.of(), 0.45);
    var mode = new ShadowMode(1.0, 0.0);
    var t = mode.compute(ctx(f, new Vec2(5, 5), opp(0, 10, 1.0, 0, 0))).orElseThrow();
    assertTrue(t.position().y() >= 0.45 - TOL);
  }

  @Test
  void rejectsBadParameters() {
    assertThrows(IllegalArgumentException.class, () -> new ShadowMode(0.0, 0.1));
    assertThrows(IllegalArgumentException.class, () -> new ShadowMode(1.0, -0.1));
  }
}
