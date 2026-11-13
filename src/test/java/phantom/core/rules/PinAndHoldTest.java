package phantom.core.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import phantom.core.control.RulesGuard;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseContext;
import phantom.core.modes.DefenseMode;
import phantom.core.modes.DefenseTarget;
import phantom.core.modes.HoldGroundMode;
import phantom.core.tracking.TrackedRobot;

/** PinTimer, HoldGroundMode and RulesGuard. */
class PinAndHoldTest {
  private static final double TOL = 1e-9;
  private static final PinRules RULES = new PinRules(3.0, 1.83);

  // ---------------- PinTimer ----------------

  @Test
  void pinTimerCountsOnlyTouchingTimeAndResetsOnSeparation() {
    var timer = new PinTimer(RULES, 0.45); // contact when centers <= 0.9 + 0.05 apart
    Vec2 self = Vec2.ZERO;
    timer.update(0.0, self, Map.of(0, new Vec2(0.9, 0))); // gap 0 -> episode starts at 0 s
    assertTrue(timer.episodeOpen(0));
    assertEquals(0.0, timer.contactSeconds(0), TOL);

    timer.update(1.0, self, Map.of(0, new Vec2(0.9, 0))); // touching 0 -> 1: +1.0
    assertEquals(1.0, timer.contactSeconds(0), TOL);

    timer.update(2.0, self, Map.of(0, new Vec2(1.5, 0))); // gap 0.6 < 1.83: paused, still open
    assertEquals(1.0, timer.contactSeconds(0), TOL);
    assertTrue(timer.episodeOpen(0));

    timer.update(3.0, self, Map.of(0, new Vec2(0.9, 0))); // re-contact: wasn't touching, no add
    assertEquals(1.0, timer.contactSeconds(0), TOL);
    timer.update(3.5, self, Map.of(0, new Vec2(0.9, 0))); // +0.5 -> 1.5
    assertEquals(1.5, timer.contactSeconds(0), TOL);

    timer.update(4.0, self, Map.of(0, new Vec2(3.0, 0))); // gap 2.1 >= 1.83 -> episode over
    assertFalse(timer.episodeOpen(0));
    assertEquals(0.0, timer.contactSeconds(0), TOL);
  }

  @Test
  void pinTimerIgnoresContactWhenNotPressing() {
    var timer = new PinTimer(RULES, 0.45);
    timer.update(0.0, Vec2.ZERO, Map.of(0, new Vec2(0.9, 0)), id -> false);
    timer.update(2.0, Vec2.ZERO, Map.of(0, new Vec2(0.9, 0)), id -> false);
    assertFalse(timer.episodeOpen(0));
    assertEquals(0.0, timer.contactSeconds(0), TOL);
  }

  // ---------------- HoldGroundMode ----------------

  /** Inner mode that always targets (2,4) against opponent 0, like a retreating Shadow would. */
  private static final DefenseMode RETREAT =
      new DefenseMode() {
        public String name() {
          return "Retreat";
        }

        public Optional<DefenseTarget> compute(DefenseContext ctx) {
          return Optional.of(new DefenseTarget(new Vec2(2, 4), Math.PI, Vec2.ZERO, 0));
        }
      };

  private static DefenseContext ctx(Vec2 goal, Vec2 self, Vec2 opp) {
    var field = new FieldConfig(20, 20, goal, List.of(), 0.45);
    var r = new TrackedRobot(0, opp, Vec2.ZERO, 0.0, 1.0);
    return new DefenseContext(0.0, new Pose(self, 0), Vec2.ZERO, List.of(r), field, 0);
  }

  @Test
  void holdPushesBackWhenTouchingAndGoalSide() {
    // self (5,4), opp (5.9,4): gap = 0.9 - 0.9 = 0 <= 0.3. goal (0,4): |self-goal| 5 < |opp-goal| 5.9
    // n = (1,0), target = (5,4) + 0.3*(1,0) = (5.3,4), v_ff = 4.5*(1,0), heading 0
    var mode = new HoldGroundMode(RETREAT, 0.3, 4.5);
    var t = mode.compute(ctx(new Vec2(0, 4), new Vec2(5, 4), new Vec2(5.9, 4))).orElseThrow();
    assertEquals(5.3, t.position().x(), TOL);
    assertEquals(4.0, t.position().y(), TOL);
    assertEquals(4.5, t.velocityFeedforward().x(), TOL);
    assertEquals(0.0, t.heading(), TOL);
  }

  @Test
  void holdDoesNothingWhenNotGoalSideOrNotClose() {
    var mode = new HoldGroundMode(RETREAT, 0.3, 4.5);
    // goal (20,4): |self-goal| 15 > |opp-goal| 14.1 -> not goal-side -> inner target (2,4)
    var notGoalSide = mode.compute(ctx(new Vec2(20, 4), new Vec2(5, 4), new Vec2(5.9, 4))).orElseThrow();
    assertEquals(2.0, notGoalSide.position().x(), TOL);
    // opp (6.5,4): gap = 1.5 - 0.9 = 0.6 > 0.3 -> inner target
    var far = mode.compute(ctx(new Vec2(0, 4), new Vec2(5, 4), new Vec2(6.5, 4))).orElseThrow();
    assertEquals(2.0, far.position().x(), TOL);
  }

  // ---------------- RulesGuard ----------------

  @Test
  void backoffStepsAsideFromAMovingOpponent() {
    // opp (5,4) moving (2,0): v_hat = (1,0), perp = (0,1)
    // self (5.9,4): away = (1,0), perp.away = 0 >= 0 -> keep (0,1)
    Vec2 d = RulesGuard.backoffDirection(new Vec2(5.9, 4), new Vec2(5, 4), new Vec2(2, 0));
    assertEquals(0.0, d.x(), TOL);
    assertEquals(1.0, d.y(), TOL);
    // self (5.5,3.5): away = (0.5,-0.5)/0.7071 = (0.7071,-0.7071), perp.away = -0.7071 < 0 -> (0,-1)
    Vec2 d2 = RulesGuard.backoffDirection(new Vec2(5.5, 3.5), new Vec2(5, 4), new Vec2(2, 0));
    assertEquals(0.0, d2.x(), TOL);
    assertEquals(-1.0, d2.y(), TOL);
  }

  @Test
  void backoffGoesStraightAwayFromAStoppedOpponent() {
    // |v| = 0.1 <= 0.3 -> away = unit((5.9,4) - (5,4)) = (1,0)
    Vec2 d = RulesGuard.backoffDirection(new Vec2(5.9, 4), new Vec2(5, 4), new Vec2(0.1, 0));
    assertEquals(1.0, d.x(), TOL);
    assertEquals(0.0, d.y(), TOL);
  }

  @Test
  void guardTakesOverBeforeTheLimit() {
    var field = new FieldConfig(20, 20, new Vec2(0, 4), List.of(), 0.45);
    var guard = new RulesGuard(RULES, field, 0.5); // trigger at 3.0 - 0.5 = 2.5 s of contact
    Pose self = new Pose(5, 4, 0);
    var opp = List.of(new TrackedRobot(0, new Vec2(5.9, 4), Vec2.ZERO, 0.0, 1.0));
    var base = Optional.of(new DefenseTarget(new Vec2(5.3, 4), 0, Vec2.ZERO, 0));

    Optional<DefenseTarget> out = base;
    for (int i = 0; i <= 100; i++) { // 0 .. 2.0 s
      out = guard.apply(i * 0.02, self, opp, base);
    }
    assertFalse(guard.backingOff());
    assertEquals(5.3, out.orElseThrow().position().x(), TOL);

    for (int i = 101; i <= 130; i++) { // to 2.6 s, past the 2.5 s trigger
      out = guard.apply(i * 0.02, self, opp, base);
    }
    assertTrue(guard.backingOff());
    assertEquals(1, guard.backoffsTriggered());
    // stopped opponent -> d = unit((5,4) - (5.9,4)) = (-1,0)
    // p* = (5,4) + 1.0*(-1,0) = (4,4), v_ff = 3.0*(-1,0)
    assertEquals(4.0, out.orElseThrow().position().x(), TOL);
    assertEquals(4.0, out.orElseThrow().position().y(), TOL);
    assertEquals(-3.0, out.orElseThrow().velocityFeedforward().x(), TOL);
  }

  @Test
  void guardBacksOffFromEveryoneOverTheLimit() {
    // Squeezed from both sides by stopped opponents: d1 = (-1,0), d2 = (1,0) -> sum 0
    // -> slide sideways: any = unit(self - nearest) = (-1,0) or (1,0); perp = (0,-1) or (0,1)
    var field = new FieldConfig(20, 20, new Vec2(0, 4), List.of(), 0.45);
    var guard = new RulesGuard(RULES, field, 0.5);
    Pose self = new Pose(5, 4, 0);
    var opps =
        List.of(
            new TrackedRobot(0, new Vec2(5.9, 4), Vec2.ZERO, 0.0, 1.0),
            new TrackedRobot(1, new Vec2(4.1, 4), Vec2.ZERO, 0.0, 1.0));
    Optional<DefenseTarget> out = Optional.empty();
    for (int i = 0; i <= 130; i++) {
      out = guard.apply(i * 0.02, self, opps, Optional.empty());
    }
    assertEquals(2, guard.backoffsTriggered());
    var t = out.orElseThrow();
    assertEquals(5.0, t.position().x(), TOL); // moved purely sideways
    assertEquals(1.0, Math.abs(t.position().y() - 4.0), TOL);
  }
}
