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
import phantom.core.tracking.TrackedRobot;

/**
 * Pin timing when the tracker changes the id of the robot we are pressing (anonymous tracking).
 *
 * <p>Setup used by every test: 10 x 10 m field, robot half size 0.45 m, pin limit 3.0 s. We stand
 * at (5,5); the opponent sits at (5.9,5): center distance 0.9 = 2 * 0.45, so gap = 0. Camera only
 * (no drive telemetry), so contact comes from the vision cue alone:
 *
 * <pre>
 *   vision = ramp(0; 0.45, 0.05) = 1        L = bias + wVision = -4 + 5.5 = 1.5
 *   raw    = 1 / (1 + e^-1.5) = 0.8175745
 * </pre>
 *
 * Loops are dt = 1/16 s = 0.0625 s (exact in binary, so the timer sums are exact). Smoothing:
 * p += (raw - p) * dt / (tauRise + dt), factor = 0.0625 / 0.0825 = 0.7575758.
 *
 * <pre>
 *   loop 0 (t = 0):      first call, dt = 0, p = 0
 *   loop 1 (t = 0.0625): p = 0.8175745 * 0.7575758 = 0.6193746            (< 0.7: not yet)
 *   loop 2 (t = 0.125):  p = 0.6193746 + (0.8175745 - 0.6193746) * 0.7575758 = 0.7695266  (enter)
 *   pin episode opens at t = 0.125 with 0 s; each later loop in contact adds 0.0625 s:
 *     contact(t) = t - 0.125
 * </pre>
 */
class IdSwitchPinTest {
  private static final double TOL = 1e-9;
  private static final double DT = 0.0625;
  private static final Vec2 SELF = new Vec2(5, 5);
  private static final Vec2 OPP = new Vec2(5.9, 5);

  private static FieldConfig field() {
    return new FieldConfig("test", 10, 10, new Vec2(9, 5), List.of(), 0.45, List.of(), List.of());
  }

  private static List<TrackedRobot> seen(int id, double t) {
    return List.of(new TrackedRobot(id, OPP, Vec2.ZERO, t, 1.0));
  }

  /** One camera-only loop at loop index k. */
  private static Optional<DefenseTarget> loop(RulesGuard g, int k, List<TrackedRobot> tracks) {
    return g.apply(k * DT, new Pose(SELF, 0.0), tracks, Optional.empty());
  }

  @Test
  void idSwitchAtTwoSecondsOfContactKeepsTheTimerAndBacksOffAtTheWarningThreshold() {
    var g = new RulesGuard(PinRules.placeholder(), field(), 0.5); // warning at 3.0 - 0.5 = 2.5 s
    // Loops 0..33 the tracker calls the robot #1000.
    for (int k = 0; k <= 33; k++) {
      loop(g, k, seen(1000, k * DT));
    }
    // loop 33: t = 2.0625, contact = 2.0625 - 0.125 = 1.9375
    assertEquals(1.9375, g.contactSeconds(1000), TOL);

    // Loop 34 (t = 2.125): same robot, same place, but the tracker now calls it #1001.
    loop(g, 34, seen(1001, 34 * DT));
    // contact = 2.125 - 0.125 = 2.0 s, NOT a fresh 0 s: #1001 continues #1000's episode
    // (new id, |(5.9,5) - (5.9,5)| = 0 <= REBIND_RADIUS 1.0 from where #1000 was last seen fresh).
    assertEquals(2.0, g.contactSeconds(1001), TOL);
    assertEquals(2.0, g.contactSeconds(1000), TOL); // the same episode, under its old name
    // The detector's belief carried over too: p has converged to raw (|p - raw| < 0.25^33),
    // so we are still in contact and the most likely opponent is the new id.
    assertTrue(g.lastContact().inContact());
    assertEquals(1001, g.lastContact().opponentId());
    assertEquals(0.8175745, g.lastContact().probability(), 1e-6);
    assertEquals(1, g.episodeRebinds());

    // Loops 35..41: still pressing #1001. Loop 41 (t = 2.5625): contact 2.4375 < 2.5, no back-off.
    for (int k = 35; k <= 41; k++) {
      loop(g, k, seen(1001, k * DT));
    }
    assertEquals(2.4375, g.contactSeconds(1001), TOL);
    assertFalse(g.backingOff());

    // Loop 42 (t = 2.625): contact = 2.625 - 0.125 = 2.5 >= 3.0 - 0.5 -> back off, from #1001.
    Optional<DefenseTarget> out = loop(g, 42, seen(1001, 42 * DT));
    assertEquals(2.5, g.contactSeconds(1001), TOL);
    assertTrue(g.backingOff());
    assertEquals(1, g.backoffsTriggered());
    assertEquals(1001, out.orElseThrow().opponentId());
    assertTrue(out.get().position().isFinite());
  }

  @Test
  void withStableIdsTheSameSwitchWouldRestartTheTimer() {
    // What the guard did before episode keys (and still does for trackers whose ids never change,
    // where a new id is a different robot): #1001 starts its own episode.
    var g = new RulesGuard(PinRules.placeholder(), field(), 0.5);
    g.setIdsAreStable(true);
    for (int k = 0; k <= 33; k++) {
      loop(g, k, seen(1000, k * DT));
    }
    loop(g, 34, seen(1001, 34 * DT)); // new hypothesis: p = 0.6193746 < 0.7, not in contact yet
    assertEquals(0.0, g.contactSeconds(1001), TOL);
    loop(g, 35, seen(1001, 35 * DT)); // p = 0.7695266: episode opens at t = 2.1875 with 0 s
    assertEquals(0.0, g.contactSeconds(1001), TOL);
    // It would back off only at t = 2.1875 + 2.5 = 4.6875 (loop 75), 4.56 s after contact began:
    // a pin. Episode keys are what prevents this.
    for (int k = 36; k <= 74; k++) {
      loop(g, k, seen(1001, k * DT));
    }
    assertFalse(g.backingOff()); // loop 74: 74 * 0.0625 - 2.1875 = 2.4375
    loop(g, 75, seen(1001, 75 * DT));
    assertTrue(g.backingOff()); // 75 * 0.0625 - 2.1875 = 2.5
  }

  @Test
  void briefLossThenANewIdResumesTheEpisode() {
    var g = new RulesGuard(PinRules.placeholder(), field(), 0.5);
    for (int k = 0; k <= 33; k++) {
      loop(g, k, seen(1000, k * DT));
    }
    // Loops 34..37 (t = 2.125 .. 2.3125): no track at all. Nothing pushes (camera only), so the
    // lost robot's time pauses at 1.9375; its episode stays open (forgotten only after 2.0 s).
    for (int k = 34; k <= 37; k++) {
      loop(g, k, List.of());
    }
    assertEquals(1.9375, g.contactSeconds(1000), TOL);
    // Loop 38 (t = 2.375): #1001 appears where #1000 was -> continues the episode. The detector's
    // belief for it restarts (the lost track's was dropped): p = 0.6193746, not in contact.
    loop(g, 38, seen(1001, 38 * DT));
    assertEquals(1.9375, g.contactSeconds(1001), TOL);
    // Loop 39 (t = 2.4375): p = 0.7695266, touching again; the first loop of a touch adds nothing.
    loop(g, 39, seen(1001, 39 * DT));
    assertEquals(1.9375, g.contactSeconds(1001), TOL);
    // From loop 40 on +0.0625 per loop: 1.9375 + 9 * 0.0625 = 2.5 at loop 48 (t = 3.0).
    for (int k = 40; k <= 47; k++) {
      loop(g, k, seen(1001, k * DT));
    }
    assertEquals(2.4375, g.contactSeconds(1001), TOL);
    assertFalse(g.backingOff());
    loop(g, 48, seen(1001, 48 * DT));
    assertEquals(2.5, g.contactSeconds(1001), TOL);
    assertTrue(g.backingOff());
  }

  @Test
  void aNewIdFarFromTheLostRobotIsADifferentRobot() {
    var g = new RulesGuard(PinRules.placeholder(), field(), 0.5);
    for (int k = 0; k <= 33; k++) {
      loop(g, k, seen(1000, k * DT));
    }
    // #1001 appears 1.5 m from where #1000 was: |(7.4,5) - (5.9,5)| = 1.5 > REBIND_RADIUS 1.0.
    // gap = 2.4 - 0.9 = 1.5 m: no vision contact, no episode of its own.
    loop(g, 34, List.of(new TrackedRobot(1001, new Vec2(7.4, 5), Vec2.ZERO, 34 * DT, 1.0)));
    assertEquals(0.0, g.contactSeconds(1001), TOL);
    assertEquals(1.9375, g.contactSeconds(1000), TOL); // #1000's episode is untouched (paused)
    assertEquals(0, g.episodeRebinds());
  }

  // ---------------- speed-dependent safety margin ----------------

  @Test
  void moveTimeByHand() {
    // d = 2 * 0.45 + 0.05 = 0.95 m (one robot width plus the contact tolerance)
    // 3 m/s, 6 m/s^2: d_acc = 9 / 12 = 0.75 < 0.95  ->  3/6 + (0.95 - 0.75)/3 = 0.5 + 0.0666667
    assertEquals(0.5666667, RulesGuard.moveSeconds(0.95, 3.0, 6.0), 1e-7);
    // 1 m/s, 2 m/s^2: d_acc = 1 / 4 = 0.25  ->  1/2 + (0.95 - 0.25)/1 = 1.2
    assertEquals(1.2, RulesGuard.moveSeconds(0.95, 1.0, 2.0), TOL);
    // 0.5 m/s, 2 m/s^2: d_acc = 0.25 / 4 = 0.0625  ->  0.5/2 + (0.95 - 0.0625)/0.5 = 0.25 + 1.775
    assertEquals(2.025, RulesGuard.moveSeconds(0.95, 0.5, 2.0), TOL);
    // 0.5 m within d_acc = 0.75: never at top speed -> sqrt(2 * 0.5 / 6) = sqrt(1/6)
    assertEquals(0.4082483, RulesGuard.moveSeconds(0.5, 3.0, 6.0), 1e-7);
    assertEquals(0.0, RulesGuard.moveSeconds(0.0, 1.0, 2.0), TOL);
    // Not a positive speed / accel, or a negative distance: it can never get there.
    assertEquals(Double.POSITIVE_INFINITY, RulesGuard.moveSeconds(0.95, 0.0, 2.0));
    assertEquals(Double.POSITIVE_INFINITY, RulesGuard.moveSeconds(0.95, 1.0, Double.NaN));
    assertEquals(Double.POSITIVE_INFINITY, RulesGuard.moveSeconds(-1.0, 1.0, 2.0));
  }

  @Test
  void safetyMarginByHand() {
    PinRules rules = PinRules.placeholder(); // 3.0 s
    // margin = 0.5 slack + latency + 0.02 loop + moveSeconds(0.95, min(v, 3), a)
    // Simulator robot: v = min(4.5, 3) = 3, a = 6, no latency: 0.5 + 0 + 0.02 + 0.5666667
    assertEquals(1.0866667, RulesGuard.safetyMarginFor(rules, 0.45, 4.5, 6.0, 0.0), 1e-7);
    assertEquals(1.0866667, RulesGuard.simMargin(rules, 0.45), 1e-7);
    // Same robot, 0.1 s latency: 1.0866667 + 0.1
    assertEquals(1.1866667, RulesGuard.safetyMarginFor(rules, 0.45, 4.5, 6.0, 0.1), 1e-7);
    // Phase 7 first test, 1 m/s, 2 m/s^2, 0.1 s: 0.5 + 0.1 + 0.02 + 1.2 = 1.82 (back off at 1.18 s)
    assertEquals(1.82, RulesGuard.safetyMarginFor(rules, 0.45, 1.0, 2.0, 0.1), 1e-9);
    // 0.5 m/s, 2 m/s^2, 0.1 s: 0.5 + 0.1 + 0.02 + 2.025 = 2.645 (back off at 0.355 s)
    assertEquals(2.645, RulesGuard.safetyMarginFor(rules, 0.45, 0.5, 2.0, 0.1), 1e-9);
    // A negative latency counts as 0: 0.5 + 0 + 0.02 + 1.2 = 1.72
    assertEquals(1.72, RulesGuard.safetyMarginFor(rules, 0.45, 1.0, 2.0, -0.5), 1e-9);
    // 0.1 m/s: d_acc = 0.0025 -> 0.05 + 0.9475/0.1 = 9.525 -> 10.145 > 3 -> clamped to the limit
    // (back off as soon as contact starts)
    assertEquals(3.0, RulesGuard.safetyMarginFor(rules, 0.45, 0.1, 2.0, 0.1), TOL);
    assertEquals(3.0, RulesGuard.safetyMarginFor(rules, 0.45, 0.0, 2.0, 0.1), TOL);
    assertEquals(3.0, RulesGuard.safetyMarginFor(rules, 0.45, Double.NaN, 2.0, 0.1), TOL);
  }

  @Test
  void slowRobotBacksOffAtItsOwnWarningThreshold() {
    double margin = RulesGuard.safetyMarginFor(PinRules.placeholder(), 0.45, 1.0, 2.0, 0.1); // 1.82
    var g = new RulesGuard(PinRules.placeholder(), field(), margin);
    // Warning threshold 3.0 - 1.82 = 1.18. contact(t) = t - 0.125:
    //   loop 20 (t = 1.25):   1.125 < 1.18 -> not yet
    //   loop 21 (t = 1.3125): 1.1875 >= 1.18 -> back off
    for (int k = 0; k <= 20; k++) {
      loop(g, k, seen(1000, k * DT));
    }
    assertFalse(g.backingOff());
    loop(g, 21, seen(1000, 21 * DT));
    assertEquals(1.1875, g.contactSeconds(1000), TOL);
    assertTrue(g.backingOff());
  }

  @Test
  void defaultGuardUsesTheSimRobotMargin() {
    // RulesGuard.defaults: margin 1.0866667 -> threshold 3.0 - 1.0866667 = 1.9133333.
    //   loop 32 (t = 2.0):    1.875 < 1.9133 -> not yet
    //   loop 33 (t = 2.0625): 1.9375 >= 1.9133 -> back off
    var g = RulesGuard.defaults(field());
    assertEquals(1.0866667, g.safetyMargin(), 1e-7);
    for (int k = 0; k <= 32; k++) {
      loop(g, k, seen(1000, k * DT));
    }
    assertFalse(g.backingOff());
    loop(g, 33, seen(1000, 33 * DT));
    assertTrue(g.backingOff());
  }
}
