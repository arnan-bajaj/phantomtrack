package phantom.core.manager;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

class TargetSelectorTest {
  private static final double TOL = 1e-9;
  private static final Vec2 GOAL = new Vec2(10, 0);

  /** Opponent observed right now (timestamp = now), so positionAt(now) = position. */
  private static TrackedRobot opp(int id, double x, double y, double vx, double vy, double now) {
    return new TrackedRobot(id, new Vec2(x, y), new Vec2(vx, vy), now, 1.0);
  }

  @Test
  void timeToGoalByHand() {
    // (6,0) moving (2,0): toGoal (4,0), closing = (2,0).(1,0) = 2 -> T = 4 / 2 = 2
    assertEquals(2.0, TargetSelector.timeToGoal(opp(0, 6, 0, 2, 0, 0), 0, GOAL), TOL);
    // (10,4) moving (0,-2): toGoal (0,-4), unit (0,-1), closing 2 -> T = 4 / 2 = 2
    assertEquals(2.0, TargetSelector.timeToGoal(opp(0, 10, 4, 0, -2, 0), 0, GOAL), TOL);
    // moving away: closing -1 < 0.5 -> not a threat
    assertTrue(Double.isInfinite(TargetSelector.timeToGoal(opp(0, 5, 0, -1, 0, 0), 0, GOAL)));
    // creeping: closing 0.4 < 0.5 -> not a threat
    assertTrue(Double.isInfinite(TargetSelector.timeToGoal(opp(0, 8, 0, 0.4, 0, 0), 0, GOAL)));
  }

  @Test
  void picksTheBiggestThreatFirst() {
    var sel = new TargetSelector(2.0, 0.5);
    // A: (6,0) v 2 -> T 2.  B: (0,0) v 2 -> T = 10/2 = 5.  -> A
    int pick = sel.select(0, List.of(opp(0, 6, 0, 2, 0, 0), opp(1, 0, 0, 2, 0, 0)), GOAL);
    assertEquals(0, pick);
    assertEquals(0, sel.switches());
  }

  @Test
  void switchesEarlyOnlyAfterMarginAndDwell() {
    var sel = new TargetSelector(2.0, 0.5);
    sel.select(0.0, List.of(opp(0, 6, 0, 2, 0, 0.0), opp(1, 0, 0, 2, 0, 0.0)), GOAL); // -> A (#0)
    // Now A: (2,0) v 2 -> T = 8/2 = 4.  B: (8,0) v 2 -> T = 2/2 = 1.  1 < 4 - 2 = 2 -> challenger
    for (double t : new double[] {0.1, 0.3, 0.5}) { // 0.4 s of challenge < 0.5 s dwell
      assertEquals(0, sel.select(t, List.of(opp(0, 2, 0, 2, 0, t), opp(1, 8, 0, 2, 0, t)), GOAL));
    }
    assertEquals(1, sel.select(0.6, List.of(opp(0, 2, 0, 2, 0, 0.6), opp(1, 8, 0, 2, 0, 0.6)), GOAL));
    assertEquals(1, sel.switches());
  }

  @Test
  void doesNotSwitchForASmallAdvantage() {
    var sel = new TargetSelector(2.0, 0.5);
    sel.select(0.0, List.of(opp(0, 6, 0, 2, 0, 0.0), opp(1, 0, 0, 2, 0, 0.0)), GOAL); // -> A
    // A: (4,0) v 2 -> T 3.  B: (6,0) v 2 -> T 2.  2 < 3 - 2 = 1? no -> stay on A forever
    for (int i = 1; i <= 50; i++) {
      double t = i * 0.1;
      assertEquals(0, sel.select(t, List.of(opp(0, 4, 0, 2, 0, t), opp(1, 6, 0, 2, 0, t)), GOAL));
    }
    assertEquals(0, sel.switches());
  }

  @Test
  void movesOnAfterTargetTurnsAway() {
    var sel = new TargetSelector(2.0, 0.5);
    sel.select(0.0, List.of(opp(0, 6, 0, 2, 0, 0.0), opp(1, 0, 0, 2, 0, 0.0)), GOAL); // -> A
    // A scored and heads back: v (-2,0) -> T infinite. B coming: (2,0) v 2 -> T 4.
    assertEquals(0, sel.select(1.0, List.of(opp(0, 8, 0, -2, 0, 1.0), opp(1, 2, 0, 2, 0, 1.0)), GOAL));
    assertEquals(0, sel.select(1.4, List.of(opp(0, 8, 0, -2, 0, 1.4), opp(1, 2, 0, 2, 0, 1.4)), GOAL));
    assertEquals(1, sel.select(1.5, List.of(opp(0, 8, 0, -2, 0, 1.5), opp(1, 2, 0, 2, 0, 1.5)), GOAL));
    assertEquals(1, sel.switches());
  }

  @Test
  void brokenTracksAreSkippedNotFatal() {
    // Audit H1: a lone NaN track used to throw a NullPointerException here (no "best", no
    // "nearest"), which would stop robot code. Now it counts as "nobody": keep the choice (-1).
    var sel = new TargetSelector(2.0, 0.5);
    var nan = opp(0, Double.NaN, 0, 2, 0, 0);
    assertEquals(-1, sel.select(0, List.of(nan), GOAL));
    // next to a good one, the good one is picked: (6,0) v 2 -> T 2
    assertEquals(1, sel.select(0.02, List.of(nan, opp(1, 6, 0, 2, 0, 0.02)), GOAL));
  }

  @Test
  void lostTargetIsReplacedAtOnceAndEmptyKeepsChoice() {
    var sel = new TargetSelector(2.0, 0.5);
    sel.select(0.0, List.of(opp(0, 6, 0, 2, 0, 0.0), opp(1, 0, 0, 2, 0, 0.0)), GOAL); // -> A
    assertEquals(0, sel.select(0.1, List.of(), GOAL)); // nothing visible: keep A
    assertEquals(1, sel.select(0.2, List.of(opp(1, 1, 0, 2, 0, 0.2)), GOAL)); // A gone, only B
    assertEquals(1, sel.switches());
  }
}
