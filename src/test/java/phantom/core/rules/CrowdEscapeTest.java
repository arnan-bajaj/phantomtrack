package phantom.core.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import phantom.core.control.RulesGuard;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseTarget;
import phantom.core.rules.EscapePlanner.Candidate;
import phantom.core.rules.EscapePlanner.Motion;
import phantom.core.rules.EscapePlanner.Option;
import phantom.core.rules.EscapePlanner.Robot;
import phantom.core.rules.EscapePlanner.Threat;
import phantom.core.tracking.TrackedRobot;

/**
 * The back-off in a crowd (EscapePlanner's crowd checks and RulesGuard's yield), on hand-built
 * layouts with robot half size 0.45 m (two bumpers touch at 0.9 m between centers).
 */
class CrowdEscapeTest {
  private static final double TOL = 1e-9;

  private static FieldConfig field() {
    return new FieldConfig("test", 10, 10, new Vec2(9, 5), List.of(), 0.45, List.of(), List.of());
  }

  private static Candidate get(List<Candidate> cs, Option o) {
    return cs.stream().filter(c -> c.option() == o).findFirst().orElseThrow();
  }

  @Test
  void driveTimeByHand() {
    // 0.95 m at 1 m/s, 2 m/s^2: d_acc = 1^2 / (2*2) = 0.25 < 0.95 -> 1/2 + (0.95 - 0.25)/1 = 1.2 s
    assertEquals(1.2, EscapePlanner.driveSeconds(0.95, 1.0, 2.0), TOL);
    // 0.16 m: 0.16 <= 0.25 -> sqrt(2 * 0.16 / 2) = sqrt(0.16) = 0.4 s
    assertEquals(0.4, EscapePlanner.driveSeconds(0.16, 1.0, 2.0), TOL);
    // instant acceleration: 1.5 m / 3 m/s = 0.5 s
    assertEquals(0.5, EscapePlanner.driveSeconds(1.5, 3.0, Double.POSITIVE_INFINITY), TOL);
    assertEquals(0.0, EscapePlanner.driveSeconds(0.0, 1.0, 2.0), TOL);
    assertEquals(0.0, EscapePlanner.driveSeconds(-1.0, 1.0, 2.0), TOL);
  }

  @Test
  void predictionByHand() {
    Vec2 self = new Vec2(5, 5);
    // Touching, tracked, nearly stopped (0.2 <= 0.3 m/s): a = unit((5,5) - (4.1,5)) = (1,0),
    // u = (1,0) * max(0.2, 1.0) = (1,0); after 0.5 s: (4.1,5) + 0.5*(1,0) = (4.6,5)
    var stalled = new Robot(new Vec2(4.1, 5), new Vec2(0.2, 0), true, true);
    Vec2 p = EscapePlanner.predict(self, stalled, 0.5, 1.0);
    assertEquals(4.6, p.x(), TOL);
    assertEquals(5.0, p.y(), TOL);
    // The same robot not touching: constant velocity, (4.1,5) + 0.5*(0.2,0) = (4.2,5)
    var passing = new Robot(new Vec2(4.1, 5), new Vec2(0.2, 0), true, false);
    assertEquals(4.2, EscapePlanner.predict(self, passing, 0.5, 1.0).x(), TOL);
    // Not tracked (only a guess): stays where it is
    var guess = new Robot(new Vec2(4.1, 5), new Vec2(0.2, 0), false, true);
    assertEquals(4.1, EscapePlanner.predict(self, guess, 0.5, 1.0).x(), TOL);
    // Touching and moving (0,2): a = (0,1), u = (0,1) * max(2, 1) = (0,2) -> (4.1, 5 + 1) = (4.1,6)
    var rubbing = new Robot(new Vec2(4.1, 5), new Vec2(0, 2), true, true);
    p = EscapePlanner.predict(self, rubbing, 0.5, 1.0);
    assertEquals(4.1, p.x(), TOL);
    assertEquals(6.0, p.y(), TOL);
  }

  // A pusher at (4.1,5) driving +x at 1 m/s into us at (5,5) (bumpers touching), and a parked
  // robot R at (5,3.2) (gap |(0,1.8)| - 0.9 = 0.9, not touching). We back off at 1 m/s, reaching
  // it at once (t_k = s_k / 1), look-ahead D = 1 m, so p_k = (5,5) + 0.1 k d, t_k = 0.1 k.
  //   a = (1,0) (1 m/s > 0.3); LEFT = (-0,1) = (0,1), RIGHT = (0,-1), BACK = away = (1,0)
  //   the pusher is predicted at (4.1,5) + t * (1,0) * max(1,1) = (4.1 + t, 5)
  private static final Vec2 SELF = new Vec2(5, 5);
  private static final List<Threat> PUSHER =
      List.of(new Threat(new Vec2(4.1, 5), new Vec2(1, 0), true));
  private static final List<Robot> PARKED =
      List.of(new Robot(new Vec2(5, 3.2), Vec2.ZERO, true, false));
  private static final Motion ONE_MPS = new Motion(1.0, Double.POSITIVE_INFINITY);

  @Test
  void crowdChecksByHand() {
    var cs = EscapePlanner.candidates(field(), SELF, PUSHER, PARKED, 1.0, ONE_MPS);

    Candidate left = get(cs, Option.LEFT);
    assertEquals(0.0, left.direction().x(), TOL);
    assertEquals(1.0, left.direction().y(), TOL);
    // (0,1) . unit((4.1,5) - (5,5)) = (0,1).(-1,0) = 0 -> does not push on the pusher
    assertFalse(left.intoRobot());
    // (0,1) . a = (0,1).(1,0) = 0 <= cos 45 deg
    assertFalse(left.alongPath());
    // R: |(5, 5 + 0.1k) - (5,3.2)| = 1.8 + 0.1k >= 1.8 -> gap >= 0.9, never meets
    assertFalse(left.meetsRobot());
    // end (5,6) at t = 1: pusher at (5.1,5): sqrt(0.1^2 + 1^2) = 1.0049876 -> gap 0.1049876;
    // R: 2.8 - 0.9 = 1.9 -> endGap = 0.1049876
    assertEquals(0.1049876, left.endGap(), 1e-7);
    // leaves the pusher's lane: |a x (end - p)| = |(1,0) x (0.9,1)| = 1 >= 0.9
    assertTrue(left.leavesPath());
    assertTrue(left.usable());

    Candidate right = get(cs, Option.RIGHT);
    // R: |(5, 5 - 0.1k) - (5,3.2)| = 1.8 - 0.1k -> gap 0.9 - 0.1k <= 0.05 from k = 9 (gap 0)
    assertTrue(right.meetsRobot());
    // end (5,4): R at 0.8 -> gap -0.1 (overlap); pusher 0.1049876 -> endGap -0.1
    assertEquals(-0.1, right.endGap(), 1e-9);
    assertFalse(right.usable());

    Candidate back = get(cs, Option.BACK);
    // (1,0) . a = 1 > cos 45 deg: straight back along the pusher's path
    assertTrue(back.alongPath());
    // end (6,5), pusher at (5.1,5): 0.9 - 0.9 = 0 -> it is still on our bumper (chased)
    assertEquals(0.0, back.endGap(), 1e-9);
    assertFalse(back.usable());

    assertEquals(Option.LEFT, EscapePlanner.chooseExit(cs, List.of()).orElseThrow().option());
  }

  @Test
  void blockedDirectionsAreSkippedWithinThirtyDegrees() {
    var cs = EscapePlanner.candidates(field(), SELF, PUSHER, PARKED, 1.0, ONE_MPS);
    // LEFT (0,1) . (0,1) = 1 > cos 30 deg = 0.866 -> blocked; RIGHT and BACK are not usable
    assertEquals(Optional.empty(), EscapePlanner.chooseExit(cs, List.of(new Vec2(0, 1))));
    // (0,1) . (0.6,0.8) = 0.8 <= 0.866 -> not blocked
    assertEquals(
        Option.LEFT,
        EscapePlanner.chooseExit(cs, List.of(new Vec2(0.6, 0.8))).orElseThrow().option());
  }

  @Test
  void pushingOnAnotherTouchingRobotIsNotAnExit() {
    // The parked robot is now touching our left side at (5,5.9) (gap 0.9 - 0.9 = 0).
    // LEFT (0,1) . unit((5,5.9) - (5,5)) = (0,1).(0,1) = 1 > 0 -> would push on it
    var touchingLeft = List.of(new Robot(new Vec2(5, 5.9), Vec2.ZERO, true, true));
    var cs = EscapePlanner.candidates(field(), SELF, PUSHER, touchingLeft, 1.0, ONE_MPS);
    assertTrue(get(cs, Option.LEFT).intoRobot());
    assertFalse(get(cs, Option.LEFT).usable());
    // RIGHT (0,-1): (0,-1).(0,1) = -1, not into it; but the stopped robot's path is taken to be
    // straight through us, a = unit((5,5) - (5,5.9)) = (0,-1), and (0,-1).(0,-1) = 1 > cos 45 deg
    assertFalse(get(cs, Option.RIGHT).intoRobot());
    assertTrue(get(cs, Option.RIGHT).alongPath());
  }

  @Test
  void extraDirectionsArePerpendicularsThenCompass() {
    // One touching tracked robot with path a = (1,0): (-a.y, a.x) = (0,1), (a.y, -a.x) = (0,-1);
    // then 16 compass directions starting at 0 deg = (1,0)
    var robots = List.of(new Robot(new Vec2(4.1, 5), new Vec2(1, 0), true, true),
        new Robot(new Vec2(8, 8), Vec2.ZERO, true, false)); // not touching: no perpendiculars
    var dirs = EscapePlanner.extraDirections(SELF, robots);
    assertEquals(2 + 16, dirs.size());
    assertEquals(1.0, dirs.get(0).y(), TOL);
    assertEquals(-1.0, dirs.get(1).y(), TOL);
    assertEquals(1.0, dirs.get(2).x(), TOL); // 0 deg
    assertEquals(1.0, dirs.get(2 + 4).y(), 1e-12); // 4 * 22.5 = 90 deg -> (0,1)
  }

  @Test
  void lookaheadByHand() {
    var guard = RulesGuard.defaults(field());
    // sim robot: max(min(4.5, 3) * 0.5, 2 * 0.45 + 0.05) = max(1.5, 0.95) = 1.5 m
    assertEquals(1.5, guard.lookahead(), TOL);
    // Phase 7 first test, 1 m/s and 2 m/s^2: max(1 * 0.5, 0.95) = 0.95 m
    guard.setDriveLimits(1.0, 2.0);
    assertEquals(0.95, guard.lookahead(), TOL);
    // nonsense limits are ignored
    guard.setDriveLimits(Double.NaN, 2.0);
    assertEquals(0.95, guard.lookahead(), TOL);
  }

  @Test
  void cornerWithNoSafeExitYieldsThenSlidesOut() {
    // We are at (5,4), touched by stopped opponents A at (5.9,4) (east) and B at (5,4.9) (north),
    // both gap 0, so both pass the 2.5 s trigger (limit 3 s, margin 0.5 s) at t = 2.56 (camera
    // contact from t = 0.06, see EscapePlannerTest). C, parked at (3.5,2.5), is 2.12 m away.
    // Sim robot: v = 3 m/s, a = 6 m/s^2, D = 1.5 m.
    //   pushing on A or B:  d . (1,0) > 0 or d . (0,1) > 0   -> need d.x <= 0 and d.y <= 0
    //   along A's path a_A = unit((5,4) - (5.9,4)) = (-1,0): -d.x > cos 45 deg -> need d.x >= -0.7071
    //   along B's path a_B = (0,-1):                           -d.y > cos 45 deg -> need d.y >= -0.7071
    //   so only the diagonal d = (-0.7071,-0.7071) is left (BACK = unit((-1,0) + (0,-1))), and it
    //   ends at (5,4) + 1.5 d = (3.9393,2.9393), 0.62 m from C: gap 0.62 - 0.9 < 0 -> meets C.
    // No safe exit -> YIELD: p* = p_self, v_ff = 0.
    var f = new FieldConfig(20, 20, new Vec2(0, 4), List.of(), 0.45);
    var guard = new RulesGuard(new PinRules(3.0, 1.83), f, 0.5);
    Pose self = new Pose(5, 4, 0);
    Optional<DefenseTarget> out = Optional.empty();
    for (int i = 0; i <= 170; i++) { // to 3.40 s: 0.84 s of yielding
      double t = i * 0.02;
      out = guard.apply(t, self, opponents(t), Optional.empty());
    }
    assertTrue(guard.backingOff());
    assertTrue(guard.yielding());
    DefenseTarget yieldTarget = out.orElseThrow();
    assertEquals(5.0, yieldTarget.position().x(), TOL);
    assertEquals(4.0, yieldTarget.position().y(), TOL);
    assertEquals(0.0, yieldTarget.velocityFeedforward().norm(), TOL);
    assertEquals(0.0, yieldTarget.heading(), TOL); // facing A, the first of the two (equally near)

    // Still wedged 1 s after the trigger (WEDGED_SECONDS): slide out at the shallowest angle.
    // leastInto: largest minAway = min(-d.x, -d.y) over all directions -> the diagonal, 0.7071
    // (BACK, first in the list). p* = (5,4) + 1.0 * (-0.7071,-0.7071) = (4.2929,3.2929),
    // v_ff = 3 * (-0.7071,-0.7071) = (-2.1213,-2.1213)
    for (int i = 171; i <= 180; i++) { // to 3.60 s
      double t = i * 0.02;
      out = guard.apply(t, self, opponents(t), Optional.empty());
    }
    assertFalse(guard.yielding());
    assertEquals(Optional.of(Option.BACK), guard.escapeOption());
    DefenseTarget slide = out.orElseThrow();
    assertEquals(5.0 - Math.sqrt(0.5), slide.position().x(), 1e-9);
    assertEquals(4.0 - Math.sqrt(0.5), slide.position().y(), 1e-9);
    assertEquals(-3.0 * Math.sqrt(0.5), slide.velocityFeedforward().x(), 1e-9);
    assertTrue(slide.position().isFinite() && slide.velocityFeedforward().isFinite());
  }

  @Test
  void aLongBackOffIsOneBackOff() {
    // Audit L1. Same corner as above, but held for 6 s. Both episodes pass the trigger at 2.56 s
    // (2 back-offs) and stay open and over it (nobody moves). The old 3 s "timeout" removed each
    // back-off at 5.56 s and the trigger re-added it in the same loop, counting it again: 4. A
    // back-off lasts while its episode is open and over the threshold; it is counted once.
    var f = new FieldConfig(20, 20, new Vec2(0, 4), List.of(), 0.45);
    var guard = new RulesGuard(new PinRules(3.0, 1.83), f, 0.5);
    Pose self = new Pose(5, 4, 0);
    for (int i = 0; i <= 300; i++) { // to 6.00 s
      double t = i * 0.02;
      guard.apply(t, self, opponents(t), Optional.empty());
    }
    assertTrue(guard.backingOff());
    assertEquals(2, guard.backoffsTriggered());
  }

  private static List<TrackedRobot> opponents(double t) {
    return List.of(
        new TrackedRobot(0, new Vec2(5.9, 4), Vec2.ZERO, t, 1.0),
        new TrackedRobot(1, new Vec2(5, 4.9), Vec2.ZERO, t, 1.0),
        new TrackedRobot(2, new Vec2(3.5, 2.5), Vec2.ZERO, t, 1.0));
  }
}
