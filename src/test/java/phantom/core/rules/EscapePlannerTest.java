package phantom.core.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.Test;
import phantom.core.control.RulesGuard;
import phantom.core.field.FieldConfig;
import phantom.core.field.Obstacle;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseTarget;
import phantom.core.rules.EscapePlanner.Option;
import phantom.core.rules.EscapePlanner.Threat;
import phantom.core.tracking.TrackedRobot;

/**
 * Wall-aware back-off on hand-built 10 x 10 m layouts, robot half size 0.45 m. Look-ahead
 * D = 3.0 m/s * 0.5 s = 1.5 m.
 */
class EscapePlannerTest {
  private static final double TOL = 1e-9;
  private static final double D = 1.5;

  private static FieldConfig field(Vec2 goal, Obstacle... walls) {
    return new FieldConfig("test", 10, 10, goal, List.of(), 0.45, List.of(walls), List.of());
  }

  /** Candidate for one option (fails the test if it is missing). */
  private static EscapePlanner.Candidate get(List<EscapePlanner.Candidate> cs, Option o) {
    return cs.stream().filter(c -> c.option() == o).findFirst().orElseThrow();
  }

  // Opponent at (4.1,5) driving +x at 2 m/s into us at (5,5):
  //   a = (1,0); LEFT = (-0,1) = (0,1); RIGHT = (0,-1); BACK = unit((5,5) - (4.1,5)) = (1,0)
  //   ends: LEFT (5,6.5), RIGHT (5,3.5), BACK (6.5,5)
  private static final Vec2 SELF = new Vec2(5, 5);
  private static final List<Threat> PUSHER =
      List.of(new Threat(new Vec2(4.1, 5), new Vec2(2, 0), true));

  @Test
  void clearanceByHand() {
    var f = field(new Vec2(9, 5), Obstacle.of(4, 4, 6, 6));
    // (3,5): walls min(3, 7, 5, 5) - 0.45 = 2.55; box dx = max(4-3, 3-6) = 1, dy = max(4-5, 5-6) = -1
    //        -> max(1,-1) - 0.45 = 0.55 -> 0.55
    assertEquals(0.55, EscapePlanner.clearance(f, new Vec2(3, 5)), TOL);
    // (3,3): dx = 1, dy = 1 -> 0.55 (square corners, same as Obstacle)
    assertEquals(0.55, EscapePlanner.clearance(f, new Vec2(3, 3)), TOL);
    // (5,5) inside the box: dx = max(-1,-1) = -1, dy = -1 -> -1.45
    assertEquals(-1.45, EscapePlanner.clearance(f, new Vec2(5, 5)), TOL);
  }

  @Test
  void hitsWallAtTheFieldEdge() {
    var f = field(new Vec2(9, 5));
    // from (1,5) going -x: end (-0.5,5), x < 0.45 -> hits
    assertTrue(EscapePlanner.hitsWall(f, new Vec2(1, 5), new Vec2(-1, 0), D));
    // going +x: end (2.5,5) -> clear
    assertFalse(EscapePlanner.hitsWall(f, new Vec2(1, 5), new Vec2(1, 0), D));
  }

  @Test
  void openFieldStepsOutOfThePathNotStraightBack() {
    var f = field(new Vec2(9, 5));
    var cs = EscapePlanner.candidates(f, SELF, PUSHER, D, List.of());
    // leaves path: |a x (end - opp)|, a = (1,0)
    //   LEFT  end - opp = (0.9, 1.5)  -> |1*1.5 - 0*0.9| = 1.5 >= 0.9 yes
    //   RIGHT end - opp = (0.9,-1.5)  -> 1.5 yes
    //   BACK  end - opp = (2.4, 0)    -> 0 no
    assertTrue(get(cs, Option.LEFT).leavesPath());
    assertTrue(get(cs, Option.RIGHT).leavesPath());
    assertFalse(get(cs, Option.BACK).leavesPath());
    // clearance (walls only): LEFT lowest at (5,6.5): min(5,5,6.5,3.5) - 0.45 = 3.05
    assertEquals(3.05, get(cs, Option.LEFT).clearance(), TOL);
    // goal (9,5): both ends sqrt(16 + 2.25) = 4.2720019 -> tie; lateral 1.5 = 1.5 -> tie -> LEFT
    assertEquals(4.2720019, get(cs, Option.LEFT).goalDistance(), 1e-7);
    var choice = EscapePlanner.choose(cs, Set.of()).orElseThrow();
    assertEquals(Option.LEFT, choice.option());
    assertEquals(0.0, choice.direction().x(), TOL); // (-a.y, a.x) = (-0, 1)
    assertEquals(1.0, choice.direction().y(), TOL);
  }

  @Test
  void wallOnTheLeftSendsUsRight() {
    // Wall (4,6.2)-(6,7): grown y from 5.75, so the LEFT drive to (5,6.5) runs into it.
    var f = field(new Vec2(9, 5), Obstacle.of(4, 6.2, 6, 7));
    var cs = EscapePlanner.candidates(f, SELF, PUSHER, D, List.of());
    assertTrue(get(cs, Option.LEFT).hitsWall());
    // LEFT clearance at (5,6.5): dx = max(4-5, 5-6) = -1, dy = max(6.2-6.5, 6.5-7) = -0.3
    //   -> -0.3 - 0.45 = -0.75 (overlap)
    assertEquals(-0.75, get(cs, Option.LEFT).clearance(), TOL);
    // RIGHT clearance is set by the wall at the start (5,5): dy = 6.2 - 5 = 1.2 -> 0.75
    assertFalse(get(cs, Option.RIGHT).hitsWall());
    assertEquals(0.75, get(cs, Option.RIGHT).clearance(), TOL);
    // LEFT excluded; RIGHT leaves the path, BACK does not -> RIGHT
    assertEquals(Option.RIGHT, EscapePlanner.choose(cs, Set.of()).orElseThrow().option());
  }

  @Test
  void goalSideBreaksATie() {
    // goal (9,8): LEFT end (5,6.5) -> sqrt(16 + 2.25) = 4.2720; RIGHT end (5,3.5) -> sqrt(16 + 20.25)
    // = 6.0208 -> LEFT keeps us nearer the goal
    var f = field(new Vec2(9, 8));
    var cs = EscapePlanner.candidates(f, SELF, PUSHER, D, List.of());
    assertEquals(6.0207973, get(cs, Option.RIGHT).goalDistance(), 1e-7);
    assertEquals(Option.LEFT, EscapePlanner.choose(cs, Set.of()).orElseThrow().option());
    // goal (9,2): mirrored -> RIGHT
    var g = field(new Vec2(9, 2));
    var cg = EscapePlanner.candidates(g, SELF, PUSHER, D, List.of());
    assertEquals(Option.RIGHT, EscapePlanner.choose(cg, Set.of()).orElseThrow().option());
  }

  @Test
  void aBlockedOptionIsSkipped() {
    var f = field(new Vec2(9, 5));
    var cs = EscapePlanner.candidates(f, SELF, PUSHER, D, List.of());
    // LEFT ruled out by the caller (we were stuck going that way) -> RIGHT
    assertEquals(Option.RIGHT, EscapePlanner.choose(cs, Set.of(Option.LEFT)).orElseThrow().option());
  }

  @Test
  void pinnedAgainstTheFieldWallBetweenTwoBoxesHasNoWayOut() {
    // We are against the left field wall at (0.45,5); a stopped opponent at (1.35,5) pushes us in.
    // away = (-1,0), a = away; LEFT = (0,-1), RIGHT = (0,1), BACK = (-1,0)
    // BACK end (-1.05,5) is off the field. Boxes (0,4)-(1,4.4) and (0,5.6)-(1,6) (grown y up to
    // 4.85 and from 5.15) block LEFT to (0.45,3.5) and RIGHT to (0.45,6.5).
    var f = field(new Vec2(9, 5), Obstacle.of(0, 4, 1, 4.4), Obstacle.of(0, 5.6, 1, 6));
    Vec2 self = new Vec2(0.45, 5);
    var threats = List.of(new Threat(new Vec2(1.35, 5), Vec2.ZERO, true));
    var cs = EscapePlanner.candidates(f, self, threats, D, List.of());
    for (var c : cs) {
      assertTrue(c.hitsWall(), c.option().toString());
    }
    assertEquals(Optional.empty(), EscapePlanner.choose(cs, Set.of()));
    assertEquals(Optional.empty(), EscapePlanner.leastInto(cs, Set.of()));
  }

  @Test
  void neverDrivesTowardAThreatButCanSlideOutWhenWedged() {
    // Wall (3,6)-(7,7) above us (grown y from 5.55); we are at (5,5.55) touching it.
    // Threats 0.9 m away along away1 = (0.6,0.8) and away2 = (-0.6,0.8):
    //   p1 = (5,5.55) - 0.9*(0.6,0.8) = (4.46,4.83), p2 = (5.54,4.83), both stopped
    // several threats: BACK = unit(away1 + away2) = unit(0,1.6) = (0,1), axis = BACK
    //   LEFT = (-1,0), RIGHT = (1,0)
    var f = field(new Vec2(9, 5), Obstacle.of(3, 6, 7, 7));
    Vec2 self = new Vec2(5, 5.55);
    var threats =
        List.of(
            new Threat(new Vec2(4.46, 4.83), Vec2.ZERO, true),
            new Threat(new Vec2(5.54, 4.83), Vec2.ZERO, true));
    var cs = EscapePlanner.candidates(f, self, threats, D, List.of());
    assertTrue(get(cs, Option.BACK).hitsWall()); // straight up into the wall
    // LEFT: min((-1,0).(0.6,0.8), (-1,0).(-0.6,0.8)) = min(-0.6, 0.6) = -0.6 -> into threat 1
    assertEquals(-0.6, get(cs, Option.LEFT).minAway(), 1e-6);
    assertTrue(get(cs, Option.LEFT).intoThreat());
    assertEquals(-0.6, get(cs, Option.RIGHT).minAway(), 1e-6);
    assertTrue(get(cs, Option.RIGHT).intoThreat());
    // sliding along the wall's bottom face (y = 5.55 is the grown edge) is not "hitting" it
    assertFalse(get(cs, Option.LEFT).hitsWall());
    assertEquals(Optional.empty(), EscapePlanner.choose(cs, Set.of()));
    // last resort: equal minAway -0.6 -> first in order, LEFT
    assertEquals(Option.LEFT, EscapePlanner.leastInto(cs, Set.of()).orElseThrow().option());
  }

  @Test
  void blindThreatPrefersStraightBack() {
    // Same pusher but not tracked (felt, not seen): leavesPath is false for all (we cannot know
    // its path), clearances all capped at 1 -> blind rule picks BACK
    var f = field(new Vec2(9, 5));
    var blind = List.of(new Threat(new Vec2(4.1, 5), Vec2.ZERO, false));
    var cs = EscapePlanner.candidates(f, SELF, blind, D, List.of());
    for (var c : cs) {
      assertFalse(c.leavesPath());
    }
    assertEquals(Option.BACK, EscapePlanner.choose(cs, Set.of()).orElseThrow().option());
  }

  // ---------------- RulesGuard with the planner ----------------

  @Test
  void guardBacksOffAwayFromTheWall() {
    // Same layout as wallOnTheLeftSendsUsRight, camera only. The opponent is re-measured every
    // loop at (4.1,5) moving (2,0). Camera contact (gap 0): L = -4 + 5.5 = 1.5, raw 0.8175745;
    // p = 0, 0.4087872, 0.6131809, 0.7153777 -> contact from t = 0.06; pin time reaches 2.5 s at
    // t = 2.56, so at 2.70 the guard is backing off.
    var f = field(new Vec2(9, 5), Obstacle.of(4, 6.2, 6, 7));
    var guard = RulesGuard.defaults(f);
    Pose self = new Pose(SELF, 0);
    Optional<DefenseTarget> out = Optional.empty();
    for (int i = 0; i <= 135; i++) {
      double t = i * 0.02;
      var opp = List.of(new TrackedRobot(0, new Vec2(4.1, 5), new Vec2(2, 0), t, 1.0));
      out = guard.apply(t, self, opp, Optional.empty());
    }
    assertTrue(guard.backingOff());
    assertEquals(Optional.of(Option.RIGHT), guard.escapeOption());
    // p* = clampInside((5,5) + 1.0*(0,-1)) = (5,4), v_ff = 3*(0,-1), heading toward opp = pi
    var t = out.orElseThrow();
    assertEquals(5.0, t.position().x(), TOL);
    assertEquals(4.0, t.position().y(), TOL);
    assertEquals(-3.0, t.velocityFeedforward().y(), TOL);
    assertEquals(Math.PI, t.heading(), TOL);
  }

  @Test
  void wallBlocksProbe() {
    var f = field(new Vec2(9, 5));
    Vec2 self = new Vec2(0.45, 5); // against the left wall
    // push (-1,0): q = clamp((0.35,5)) = (0.45,5), progress 0 / 0.1 = 0 <= 0.3 -> wall
    assertTrue(RulesGuard.wallBlocks(f, self, new Vec2(-1, 0)));
    // push (0,1): q = (0.45,5.1), progress 0.1 / 0.1 = 1 -> not a wall
    assertFalse(RulesGuard.wallBlocks(f, self, new Vec2(0, 1)));
    // push (-0.6,0.8): q = clamp((0.39,5.08)) = (0.45,5.08); (0,0.08).(-0.6,0.8) = 0.064
    //   -> 0.64 > 0.3 -> not a wall (we can slide along it)
    assertFalse(RulesGuard.wallBlocks(f, self, new Vec2(-0.6, 0.8)));
  }

  @Test
  void pinTimeMovesFromUnknownToTheOpponentWhenTheCameraFindsIt() {
    var f = field(new Vec2(9, 5));
    var guard = RulesGuard.defaults(f);
    Pose self = new Pose(SELF, 0);
    // Stalled push (3,0), nobody tracked: UNKNOWN contact from t = 0.04 (see ContactDetectorTest).
    // Pin time: 0 at 0.04, +0.02 per loop -> 1.48 s at t = 1.52.
    for (int i = 0; i <= 76; i++) {
      double t = i * 0.02;
      guard.apply(t, self, List.of(), Optional.empty(),
          new DriveTelemetry(t, new Vec2(3, 0), Vec2.ZERO, Vec2.ZERO, 60));
    }
    assertEquals(1.48, guard.contactSeconds(ContactDetector.UNKNOWN), 1e-9);
    // From t = 1.54 the camera sees #0 right in front (gap 0). Its hypothesis starts at p = 0:
    //   L = -4 + 5.5 + 7 = 8.5 -> raw 0.9997965; p = 0.4998983 at 1.54, 0.7498474 at 1.56 (enter)
    // UNKNOWN is now explained (raw 0.018) and falls by 1/6 of the gap per loop, so it is still in
    // contact (p 0.80, then 0.67 >= 0.3) and keeps counting: 1.50 at 1.54.
    // At 1.56 #0 is a new touch and UNKNOWN touched within 0.5 s -> #0 inherits UNKNOWN's 1.50 s
    // (UNKNOWN counts on to 1.52). A first touch adds nothing, so #0 = 1.50 s.
    for (int i = 77; i <= 78; i++) {
      double t = i * 0.02;
      var opp = List.of(new TrackedRobot(0, new Vec2(5.9, 5), Vec2.ZERO, t, 1.0));
      guard.apply(t, self, opp, Optional.empty(),
          new DriveTelemetry(t, new Vec2(3, 0), Vec2.ZERO, Vec2.ZERO, 60));
    }
    assertEquals(1.50, guard.contactSeconds(0), 1e-9);
    assertEquals(0, guard.lastContact().opponentId()); // p 0.75 > UNKNOWN's 0.67
  }

  @Test
  void pinTimerContactApi() {
    var timer = new PinTimer(new PinRules(3.0, 1.83), 0.45);
    Set<Integer> ids = Set.of(0);
    timer.updateContacts(0.0, ids, id -> true, id -> false); // new episode at 0 s
    assertEquals(0.0, timer.contactSeconds(0), TOL);
    timer.updateContacts(0.5, ids, id -> true, id -> false); // +0.5
    assertEquals(0.5, timer.contactSeconds(0), TOL);
    timer.updateContacts(1.0, ids, id -> false, id -> false); // apart but not separated: paused
    assertEquals(0.5, timer.contactSeconds(0), TOL);
    assertEquals(0.5, timer.lastContactSec(0), TOL);
    timer.raiseTo(0, 2.0); // max(0.5, 2.0)
    assertEquals(2.0, timer.contactSeconds(0), TOL);
    timer.raiseTo(0, 1.0); // max(2.0, 1.0): never lowers
    assertEquals(2.0, timer.contactSeconds(0), TOL);
    timer.raiseTo(7, 0.0); // 0 s: no episode opened
    assertEquals(Set.of(0), timer.openEpisodes());
    timer.updateContacts(1.5, ids, id -> false, id -> true); // separated: cleared
    assertFalse(timer.episodeOpen(0));
    assertTrue(Double.isNaN(timer.lastContactSec(0)));
  }
}
