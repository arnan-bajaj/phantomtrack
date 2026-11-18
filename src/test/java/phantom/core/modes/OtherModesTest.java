package phantom.core.modes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/** Interpose, Zone and Line mirror. */
class OtherModesTest {
  private static final double TOL = 1e-9;

  private static FieldConfig field(Vec2 goal) {
    return new FieldConfig(20, 20, goal, List.of(), 0.45);
  }

  private static DefenseContext ctx(FieldConfig f, Vec2 self, TrackedRobot... opps) {
    return new DefenseContext(0.0, new Pose(self, 0.0), Vec2.ZERO, List.of(opps), f, -1);
  }

  private static TrackedRobot opp(int id, double x, double y, double vx, double vy) {
    return new TrackedRobot(id, new Vec2(x, y), new Vec2(vx, vy), 0.0, 1.0);
  }

  // ---------------- Interpose ----------------

  @Test
  void interposeHalfwayWithScaledFeedforward() {
    // opp (2,4), goal (12,4), lambda 0.5: L = 10, dist = max(5, 1.1) = 5 -> target (7,4)
    // v_ff = (1 - 0.5) * (2,0) = (1,0)
    var mode = new InterposeMode(0.5, 1.1, 0.0);
    var t = mode.compute(ctx(field(new Vec2(12, 4)), new Vec2(15, 15), opp(0, 2, 4, 2, 0))).orElseThrow();
    assertEquals(7.0, t.position().x(), TOL);
    assertEquals(4.0, t.position().y(), TOL);
    assertEquals(1.0, t.velocityFeedforward().x(), TOL);
    assertEquals(0.0, t.velocityFeedforward().y(), TOL);
    assertEquals(Math.PI, t.heading(), TOL); // facing the opponent in -x
  }

  @Test
  void interposeKeepsMinimumStandoff() {
    // opp (10,4), goal (12,4), lambda 0.5, min 1.5: L = 2, lambda*L = 1 < 1.5 -> dist 1.5 -> (11.5,4)
    var mode = new InterposeMode(0.5, 1.5, 0.0);
    var t = mode.compute(ctx(field(new Vec2(12, 4)), new Vec2(15, 15), opp(0, 10, 4, 0, 0))).orElseThrow();
    assertEquals(11.5, t.position().x(), TOL);
    assertEquals(4.0, t.position().y(), TOL);
  }

  @Test
  void interposeNeverPastGoal() {
    // opp (11.5,4), goal (12,4), min 1.5: L = 0.5, dist = min(max(0.25, 1.5), 0.5) = 0.5 -> (12,4)
    var mode = new InterposeMode(0.5, 1.5, 0.0);
    var t = mode.compute(ctx(field(new Vec2(12, 4)), new Vec2(15, 15), opp(0, 11.5, 4, 0, 0))).orElseThrow();
    assertEquals(12.0, t.position().x(), TOL);
    assertEquals(4.0, t.position().y(), TOL);
  }

  @Test
  void interposeKeepsTheStandoffFromAnOpponentDrivingAway() {
    // Audit M1. opp (10,4) moving (-4,0) away from goal (12,4); lambda 0.5, min 1.1, lead 0.25.
    // Old: p_hat = (9,4), L = 3, dist = max(1.5, 1.1) = 1.5 -> target (10.5,4): 0.5 m from the
    // opponent, under minStandoff 1.1 (and inside its bumpers).
    // Now: u0 = (1,0), v.u0 = -4 -> v_lead = (0,0) -> p_hat = (10,4), L = 2,
    //   dist = min(max(0.5 * 2, 1.1), 2) = 1.1 -> target (11.1,4); v_ff = 0.5 * (-4,0) = (-2,0)
    var mode = new InterposeMode(0.5, 1.1, 0.25);
    var t = mode.compute(ctx(field(new Vec2(12, 4)), new Vec2(15, 15), opp(0, 10, 4, -4, 0))).orElseThrow();
    assertEquals(11.1, t.position().x(), TOL);
    assertEquals(4.0, t.position().y(), TOL);
    assertEquals(-2.0, t.velocityFeedforward().x(), TOL);
  }

  @Test
  void interposeRejectsBadLambda() {
    assertThrows(IllegalArgumentException.class, () -> new InterposeMode(1.5, 1.0, 0.0));
  }

  // ---------------- Zone ----------------

  @Test
  void zoneWaitsAtAnchorWhenNobodyIsClose() {
    // 20x20 field, goal (16,10): center (10,10), dir = (-6,0)/6 = (-1,0), anchor = (16,10) + 2*(-1,0) = (14,10)
    var mode = new ZoneMode(4.0, 1.0, 2.0, 1.0, 4.5);
    var t = mode.compute(ctx(field(new Vec2(16, 10)), new Vec2(14, 10), opp(0, 2, 2, 0, 0))).orElseThrow();
    assertEquals(14.0, t.position().x(), TOL);
    assertEquals(10.0, t.position().y(), TOL);
    assertEquals(-1, t.opponentId());
  }

  @Test
  void zoneMeetsStationaryThreatGoalSide() {
    // goal (16,10), opp (12,10) is 4 m away <= R + margin = 5 -> threat
    // stationary -> meet point q = (12,10); toGoal = (4,0), target = (12,10) + 1*(1,0) = (13,10)
    var mode = new ZoneMode(4.0, 1.0, 2.0, 1.0, 4.5);
    var t = mode.compute(ctx(field(new Vec2(16, 10)), new Vec2(14, 12), opp(3, 12, 10, 0, 0))).orElseThrow();
    assertEquals(13.0, t.position().x(), TOL);
    assertEquals(10.0, t.position().y(), TOL);
    assertEquals(3, t.opponentId());
  }

  @Test
  void zoneLeadsAMovingThreat() {
    // goal (16,10), self (16,6), opp (12,10) moving (0,-1), our speed 5
    // r = (12,10) - (16,6) = (-4,4), v = (0,-1): a = 1 - 25 = -24, b = 2*(0 + -4) = -8, c = 32
    // disc = 64 + 4*24*32 = 64 + 3072 = 3136, sqrt = 56
    // roots = (8 -+ 56) / -48 -> (8 - 56)/-48 = 1, (8 + 56)/-48 = -1.333 -> t = 1
    // q = (12,10) + 1*(0,-1) = (12,9); toGoal = (4,1), |toGoal| = sqrt(17)
    // target = (12,9) + (4,1)/sqrt(17)   (standoff 1)
    var mode = new ZoneMode(4.0, 1.0, 2.0, 1.0, 5.0);
    var t = mode.compute(ctx(field(new Vec2(16, 10)), new Vec2(16, 6), opp(0, 12, 10, 0, -1))).orElseThrow();
    double s17 = Math.sqrt(17);
    assertEquals(12.0 + 4.0 / s17, t.position().x(), TOL);
    assertEquals(9.0 + 1.0 / s17, t.position().y(), TOL);
  }

  @Test
  void zoneTargetStaysInsideCircle() {
    // goal (16,10), R = 4, margin 1. opp (11.2,10) is 4.8 m out <= 5 -> threat.
    // standoff 0 -> target = q = (11.2,10), 4.8 m from goal > R
    // -> pulled onto the circle: (16,10) + 4*(-1,0) = (12,10)
    var mode = new ZoneMode(4.0, 1.0, 2.0, 0.0, 4.5);
    var t = mode.compute(ctx(field(new Vec2(16, 10)), new Vec2(14, 10), opp(0, 11.2, 10, 0, 0))).orElseThrow();
    assertEquals(12.0, t.position().x(), TOL);
    assertEquals(10.0, t.position().y(), TOL);
  }

  @Test
  void zoneUntouchableThreatGivesFiniteTarget() {
    // Opponent at 8 m/s running away from us: no intercept -> falls back to its current position
    var mode = new ZoneMode(4.0, 1.0, 2.0, 1.0, 4.5);
    var t = mode.compute(ctx(field(new Vec2(16, 10)), new Vec2(10, 10), opp(0, 13, 10, 8, 0))).orElseThrow();
    assertTrue(t.position().isFinite());
    assertTrue(Double.isFinite(t.heading()));
  }

  // ---------------- Line mirror ----------------

  @Test
  void lineMirrorProjectsOpponentOntoLine() {
    // 20x20 field, goal (18,10): outward d = (10,10)-(18,10) = (-8,0) -> (-1,0)
    // L0 = (18,10) + 3*(-1,0) = (15,10), t = (-d.y, d.x) = (0,-1)
    // opp (5,13): (p - L0) = (-10,3), dot t = -3, target = (15,10) + (-3)*(0,-1) = (15,13)
    // v = (2,1): v.t = -1, v_ff = -1*(0,-1) = (0,1)
    var mode = new LineMirrorMode(3.0, 0.0);
    var t = mode.compute(ctx(field(new Vec2(18, 10)), new Vec2(15, 10), opp(0, 5, 13, 2, 1))).orElseThrow();
    assertEquals(15.0, t.position().x(), TOL);
    assertEquals(13.0, t.position().y(), TOL);
    assertEquals(0.0, t.velocityFeedforward().x(), TOL);
    assertEquals(1.0, t.velocityFeedforward().y(), TOL);
    assertEquals(Math.PI, t.heading(), TOL); // opponent is straight -x from (15,13)
  }

  @Test
  void lineMirrorClampsToFieldEnds() {
    // opp at y = 25 (off a 20 m field) -> projection y = 25 -> clamped to 20 - 0.45 = 19.55
    var mode = new LineMirrorMode(3.0, 0.0);
    var t = mode.compute(ctx(field(new Vec2(18, 10)), new Vec2(15, 10), opp(0, 5, 25, 0, 0))).orElseThrow();
    assertEquals(19.55, t.position().y(), TOL);
  }

  @Test
  void lineMirrorOpponentOnTheLineHasFiniteHeading() {
    var mode = new LineMirrorMode(3.0, 0.0);
    var t = mode.compute(ctx(field(new Vec2(18, 10)), new Vec2(15, 10), opp(0, 15, 12, 0, 0))).orElseThrow();
    assertTrue(Double.isFinite(t.heading()));
    assertEquals(Math.PI, t.heading(), TOL); // falls back to facing outward (-x)
  }
}
