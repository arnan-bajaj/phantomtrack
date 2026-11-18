package phantom.core.tracking;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static phantom.core.tracking.AllianceLabel.OPPONENT;
import static phantom.core.tracking.AllianceLabel.PARTNER;
import static phantom.core.tracking.AllianceLabel.UNKNOWN;
import static phantom.core.tracking.MOfN.Decision.CONFIRMED;
import static phantom.core.tracking.MOfN.Decision.PENDING;
import static phantom.core.tracking.MOfN.Decision.REJECTED;

import org.junit.jupiter.api.Test;

/** ColorBelief (log-odds alliance color) and MOfN (track confirmation), by hand. */
class ColorAndConfirmationTest {
  private static final double TOL = 1e-9;

  // ---------------- ColorBelief ----------------

  @Test
  void logOddsSequenceByHand() {
    // error rate e = 0.1: every label moves L by ln(0.9 / 0.1) = ln 9 = 2.1972246
    var b = new ColorBelief(0.5); // L = ln(0.5 / 0.5) = 0 -> P = 0.5
    assertEquals(0.0, b.logOdds(), TOL);
    assertEquals(0.5, b.probabilityOpponent(), TOL);

    b.update(OPPONENT, 0.1); // L = 2.1972246, odds 9:1 -> P = 9/10
    assertEquals(2.1972246, b.logOdds(), 1e-7);
    assertEquals(0.9, b.probabilityOpponent(), TOL);

    b.update(OPPONENT, 0.1); // L = 4.3944492, odds 81:1 -> P = 81/82 = 0.9878049
    assertEquals(4.3944492, b.logOdds(), 1e-7);
    assertEquals(81.0 / 82.0, b.probabilityOpponent(), TOL);

    b.update(PARTNER, 0.1); // one contradicting label cancels one: L = 2.1972246 -> P = 0.9
    assertEquals(0.9, b.probabilityOpponent(), TOL);

    b.update(UNKNOWN, 0.1); // no information: unchanged
    assertEquals(0.9, b.probabilityOpponent(), TOL);

    b.update(PARTNER, 0.1); // L = 0 -> P = 0.5
    assertEquals(0.5, b.probabilityOpponent(), TOL);

    b.update(PARTNER, 0.1); // L = -2.1972246, odds 1:9 -> P = 1/10
    assertEquals(-2.1972246, b.logOdds(), 1e-7);
    assertEquals(0.1, b.probabilityOpponent(), TOL);
  }

  @Test
  void priorSetsTheStartingPoint() {
    // prior 0.75: L = ln(0.75 / 0.25) = ln 3 = 1.0986123
    var b = new ColorBelief(0.75);
    assertEquals(1.0986123, b.logOdds(), 1e-7);
    assertEquals(0.75, b.probabilityOpponent(), TOL);
    assertThrows(IllegalArgumentException.class, () -> new ColorBelief(0.0));
    assertThrows(IllegalArgumentException.class, () -> new ColorBelief(1.0));
    assertThrows(IllegalArgumentException.class, () -> new ColorBelief(Double.NaN));
  }

  @Test
  void beliefSaturatesSoItCanStillBeOverturned() {
    // e = 0.1: five OPPONENT labels would give 5 * 2.1972 = 10.986, clamped to 10
    var b = new ColorBelief(0.5);
    for (int i = 0; i < 5; i++) {
      b.update(OPPONENT, 0.1);
    }
    assertEquals(10.0, b.logOdds(), TOL);
    // P = 1 / (1 + e^-10) = 1 / 1.0000454 = 0.9999546
    assertEquals(0.9999546, b.probabilityOpponent(), 1e-7);
    for (int i = 0; i < 1000; i++) {
      b.update(OPPONENT, 0.1); // stays at the clamp no matter how long we watch
    }
    assertEquals(10.0, b.logOdds(), TOL);
  }

  @Test
  void flipsNeededFromSaturationMatchTheClassComment() {
    // e = 0.05: step = ln(0.95 / 0.05) = ln 19 = 2.9444390
    //   after 3 PARTNER labels: 10 - 3 * 2.9444 = 1.1667 > 0 (still "opponent")
    //   after 4:                10 - 4 * 2.9444 = -1.7778 < 0 (flipped)
    var realistic = saturatedOpponent(0.05);
    realistic.update(PARTNER, 0.05);
    realistic.update(PARTNER, 0.05);
    realistic.update(PARTNER, 0.05);
    assertEquals(1.1666830, realistic.logOdds(), 1e-6);
    assertTrue(realistic.probabilityOpponent() > 0.5);
    realistic.update(PARTNER, 0.05);
    assertEquals(-1.7777560, realistic.logOdds(), 1e-6);
    assertTrue(realistic.probabilityOpponent() < 0.5);

    // e = 0.15: step = ln(0.85 / 0.15) = ln 5.6667 = 1.7346011 (ln 17 - ln 3)
    //   after 5: 10 - 5 * 1.7346 = 1.3270 > 0;   after 6: 10 - 6 * 1.7346 = -0.4076 < 0
    var bad = saturatedOpponent(0.15);
    for (int i = 0; i < 5; i++) {
      bad.update(PARTNER, 0.15);
    }
    assertEquals(1.3269947, bad.logOdds(), 1e-6);
    bad.update(PARTNER, 0.15);
    assertEquals(-0.4076063, bad.logOdds(), 1e-6);
  }

  private static ColorBelief saturatedOpponent(double e) {
    var b = new ColorBelief(0.5);
    for (int i = 0; i < 50; i++) {
      b.update(OPPONENT, e);
    }
    assertEquals(ColorBelief.MAX_LOG_ODDS, b.logOdds(), TOL);
    return b;
  }

  @Test
  void labelLikelihoodByHand() {
    // P(opp) = 0.9 (one OPPONENT label at e = 0.1), e = 0.1:
    //   P(label OPPONENT) = 0.9 * 0.9 + 0.1 * 0.1 = 0.81 + 0.01 = 0.82
    //   P(label PARTNER)  = 0.9 * 0.1 + 0.1 * 0.9 = 0.09 + 0.09 = 0.18
    var b = new ColorBelief(0.5);
    b.update(OPPONENT, 0.1);
    assertEquals(0.82, b.likelihood(OPPONENT, 0.1), TOL);
    assertEquals(0.18, b.likelihood(PARTNER, 0.1), TOL);
    assertEquals(1.0, b.likelihood(UNKNOWN, 0.1), TOL);
    // undecided track (P = 0.5): both labels equally likely, 0.5 * 0.9 + 0.5 * 0.1 = 0.5
    var fresh = new ColorBelief(0.5);
    assertEquals(0.5, fresh.likelihood(OPPONENT, 0.1), TOL);
    assertEquals(0.5, fresh.likelihood(PARTNER, 0.1), TOL);
  }

  @Test
  void errorRateEdgeCases() {
    // e = 0.5: labels are coin flips -> step ln(0.5 / 0.5) = 0, belief never moves
    assertEquals(0.0, ColorBelief.step(0.5), TOL);
    // e above 0.5 or NaN is treated as 0.5 (never trust an "inverted" or unknown detector)
    assertEquals(0.0, ColorBelief.step(0.7), TOL);
    assertEquals(0.0, ColorBelief.step(Double.NaN), TOL);
    // e = 0 is treated as 0.001: step = ln(0.999 / 0.001) = ln 999 = 6.9067548 (finite)
    assertEquals(6.9067548, ColorBelief.step(0.0), 1e-7);
    var b = new ColorBelief(0.5);
    b.update(PARTNER, 0.0);
    assertTrue(b.likelihood(OPPONENT, 0.0) > 0.0); // never exactly 0, so -2 ln(...) stays finite
  }

  // ---------------- MOfN ----------------

  private static MOfN.Decision[] run(int m, int n, boolean... hits) {
    var logic = new MOfN(m, n);
    MOfN.Decision[] out = new MOfN.Decision[hits.length];
    for (int i = 0; i < hits.length; i++) {
      out[i] = logic.add(hits[i]);
    }
    return out;
  }

  private static final boolean H = true;
  private static final boolean M = false;

  @Test
  void threeOfFiveConfirmationSequences() {
    // H H H: hits 1, 2, 3 -> confirmed at frame 3
    assertEquals(java.util.List.of(PENDING, PENDING, CONFIRMED),
        java.util.List.of(run(3, 5, H, H, H)));
    // H M H M H: hits 1,1,2,2,3; misses 0,1,1,2,2 (never above 5 - 3 = 2) -> confirmed at frame 5
    assertEquals(java.util.List.of(PENDING, PENDING, PENDING, PENDING, CONFIRMED),
        java.util.List.of(run(3, 5, H, M, H, M, H)));
    // H M M H H: hits 1,1,1,2,3 -> confirmed at frame 5
    assertEquals(java.util.List.of(PENDING, PENDING, PENDING, PENDING, CONFIRMED),
        java.util.List.of(run(3, 5, H, M, M, H, H)));
    // H M M M: misses 3 > 2 at frame 4 -> rejected (best case would be 2 hits in 5 frames)
    assertEquals(java.util.List.of(PENDING, PENDING, PENDING, REJECTED),
        java.util.List.of(run(3, 5, H, M, M, M)));
    // H H M M M: hits 2, misses 3 > 2 at frame 5 -> rejected
    assertEquals(java.util.List.of(PENDING, PENDING, PENDING, PENDING, REJECTED),
        java.util.List.of(run(3, 5, H, H, M, M, M)));
    // H M H M M: hits 2, misses 3 at frame 5 -> rejected
    assertEquals(java.util.List.of(PENDING, PENDING, PENDING, PENDING, REJECTED),
        java.util.List.of(run(3, 5, H, M, H, M, M)));
  }

  @Test
  void decisionIsFinal() {
    // confirmed, then misses forever: still confirmed (deleting old tracks is the timeout's job)
    assertEquals(java.util.List.of(PENDING, PENDING, CONFIRMED, CONFIRMED, CONFIRMED),
        java.util.List.of(run(3, 5, H, H, H, M, M)));
    // rejected, then a hit: still rejected
    assertEquals(java.util.List.of(PENDING, PENDING, PENDING, REJECTED, REJECTED),
        java.util.List.of(run(3, 5, H, M, M, M, H)));
  }

  @Test
  void otherSizes() {
    // 1 of 1: the first detection is enough
    assertEquals(java.util.List.of(CONFIRMED), java.util.List.of(run(1, 1, H)));
    // 2 of 2: one miss is already too many (1 > 2 - 2 = 0)
    assertEquals(java.util.List.of(PENDING, REJECTED), java.util.List.of(run(2, 2, H, M)));
    // 2 of 3: H M H -> confirmed at frame 3
    assertEquals(java.util.List.of(PENDING, PENDING, CONFIRMED),
        java.util.List.of(run(2, 3, H, M, H)));
    assertThrows(IllegalArgumentException.class, () -> new MOfN(0, 5));
    assertThrows(IllegalArgumentException.class, () -> new MOfN(4, 3));
  }
}
