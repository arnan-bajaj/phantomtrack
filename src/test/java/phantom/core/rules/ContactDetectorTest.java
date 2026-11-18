package phantom.core.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * ContactDetector with the placeholder config: pushMin 0.4, stall ramp 0.60 -> 0.25, freeAccel 3,
 * current 20 -> 40 A, decel 8 -> 20 m/s^2, vision gap 0.45 -> 0.05, bias -4, weights 3.5 / 3.5 /
 * 3.5 / 5.5, tauRise 0.02, tauFall 0.10, enter 0.7, exit 0.3. Robot half size 0.45.
 */
class ContactDetectorTest {
  private static final double TOL = 1e-6;
  private static final Vec2 SELF = new Vec2(5, 5);

  private static ContactDetector detector() {
    return new ContactDetector(ContactDetector.Config.placeholder(), 0.45);
  }

  private static DriveTelemetry tel(double t, Vec2 cmd, Vec2 v, Vec2 a, double amps) {
    return new DriveTelemetry(t, cmd, v, a, amps);
  }

  /** Pushing (3,0) at full current and not moving. */
  private static DriveTelemetry stalled(double t) {
    return tel(t, new Vec2(3, 0), Vec2.ZERO, Vec2.ZERO, 60);
  }

  private static TrackedRobot opp(int id, Vec2 p, double t) {
    return new TrackedRobot(id, p, Vec2.ZERO, t, 1.0);
  }

  @Test
  void rampAndSigmoid() {
    assertEquals(0.5, ContactDetector.ramp(0.5, 0, 1), TOL);
    assertEquals(0.0, ContactDetector.ramp(-1, 0, 1), TOL); // clamped
    assertEquals(1.0, ContactDetector.ramp(7, 0, 1), TOL); // clamped
    // falling ramp: (0.4 - 0.6) / (0.25 - 0.6) = -0.2 / -0.35 = 0.5714286
    assertEquals(0.5714286, ContactDetector.ramp(0.4, 0.6, 0.25), TOL);
    assertEquals(0.5, ContactDetector.sigmoid(0), TOL);
    // 1 / (1 + e^-1.5) = 1 / 1.2231302 = 0.8175745
    assertEquals(0.8175745, ContactDetector.sigmoid(1.5), TOL);
  }

  @Test
  void stalledPushLightsStallAndCurrent() {
    var d = detector();
    // progress = (0.3,0).(1,0) / 3 = 0.1 -> ramp(0.1; 0.6, 0.25) = (0.1-0.6)/(-0.35) = 1.43 -> 1
    // speedingUp = ramp(0; 0, 3) = 0 -> stall = 1 * (1 - 0) = 1
    var t = tel(0, new Vec2(3, 0), new Vec2(0.3, 0), Vec2.ZERO, 60);
    assertEquals(1.0, d.stallCue(t), TOL);
    // current: ramp(60; 20, 40) = 2 -> 1
    assertEquals(1.0, d.currentCue(t), TOL);
    // no deceleration: ramp(-0; 8, 20) = 0
    assertEquals(0.0, d.decelCueNow(t), TOL);
  }

  @Test
  void speedingUpDiscountsStallAndCurrent() {
    var d = detector();
    // v = (1.2,0): progress = 1.2/3 = 0.4 -> (0.4-0.6)/(-0.35) = 0.5714286
    // a = (1.5,0): speedingUp = ramp(1.5; 0, 3) = 0.5
    // stall = 0.5714286 * (1 - 0.5) = 0.2857143
    var t = tel(0, new Vec2(3, 0), new Vec2(1.2, 0), new Vec2(1.5, 0), 30);
    assertEquals(0.2857143, d.stallCue(t), TOL);
    // current: ramp(30; 20, 40) = 0.5, * (1 - 0.5) = 0.25
    assertEquals(0.25, d.currentCue(t), TOL);
  }

  @Test
  void suddenDecelerationIsAHit() {
    var d = detector();
    // cmd (3,0), v (1,0) slower than asked, a (-14,0): ramp(14; 8, 20) = 6/12 = 0.5
    var t = tel(0, new Vec2(3, 0), new Vec2(1, 0), new Vec2(-14, 0), 10);
    assertEquals(0.5, d.decelCueNow(t), TOL);
  }

  @Test
  void brakingIsNotContact() {
    var d = detector();
    // asked 2 m/s, moving 3 m/s: v.u = 3 > 2 -> braking: current and decel cues are 0
    var t = tel(0, new Vec2(2, 0), new Vec2(3, 0), new Vec2(-15, 0), 60);
    assertTrue(ContactDetector.braking(t));
    assertEquals(0.0, d.currentCue(t), TOL);
    assertEquals(0.0, d.decelCueNow(t), TOL);
  }

  @Test
  void notPushingMeansNoPhysicalCues() {
    var d = detector();
    // |cmd| = 0.3 < 0.4 m/s
    var t = tel(0, new Vec2(0.3, 0), Vec2.ZERO, new Vec2(-30, 0), 60);
    assertEquals(0.0, d.stallCue(t), TOL);
    assertEquals(0.0, d.currentCue(t), TOL);
    assertEquals(0.0, d.decelCueNow(t), TOL);
  }

  @Test
  void missingCurrentFallsBackToStall() {
    var d = detector();
    var t = tel(0, new Vec2(3, 0), new Vec2(1.2, 0), new Vec2(1.5, 0), Double.NaN);
    assertFalse(t.hasCurrent());
    assertEquals(d.stallCue(t), d.currentCue(t), TOL); // 0.2857143, see speedingUp test
  }

  @Test
  void visionCue() {
    var d = detector();
    // ramp(gap; 0.45, 0.05): gap 0 -> 1.125 -> 1, gap 0.25 -> (0.25-0.45)/(-0.4) = 0.5, gap 1 -> 0
    assertEquals(1.0, d.visionCue(0.0), TOL);
    assertEquals(0.5, d.visionCue(0.25), TOL);
    assertEquals(0.0, d.visionCue(1.0), TOL);
    // withVisionError(0.2): far = 0.05 + 0.2 = 0.25 -> gap 0.15 gives (0.15-0.25)/(0.05-0.25) = 0.5
    var tight =
        new ContactDetector(ContactDetector.Config.placeholder().withVisionError(0.2), 0.45);
    assertEquals(0.5, tight.visionCue(0.15), TOL);
  }

  @Test
  void pushingIntoATrackedOpponentEntersContactWithHysteresis() {
    var d = detector();
    Vec2 oppPos = new Vec2(5.9, 5); // gap = 0.9 - 0.9 = 0, dead ahead of the push
    // vision 1; explains (gap 0 <= 0.6, cos 1 >= 0); physical = 3.5*1 + 3.5*1 + 3.5*0 = 7
    // L_0 = -4 + 5.5 + 7 = 8.5 -> raw 0.9997965
    // smoothing going up: p += (raw - p) * 0.02 / (0.02 + 0.02) = (raw - p) / 2
    var e0 = d.update(0.00, stalled(0.00), SELF, List.of(opp(0, oppPos, 0.00))); // dt 0: p = 0
    assertFalse(e0.inContact());
    var e1 = d.update(0.02, stalled(0.02), SELF, List.of(opp(0, oppPos, 0.02)));
    assertEquals(0.4998983, e1.probabilities().get(0), TOL); // 0.9997965 / 2
    assertFalse(e1.inContact()); // < 0.7
    var e2 = d.update(0.04, stalled(0.04), SELF, List.of(opp(0, oppPos, 0.04)));
    // 0.4998983 + (0.9997965 - 0.4998983) / 2 = 0.7498474 >= 0.7 -> enter
    assertEquals(0.7498474, e2.probabilities().get(0), TOL);
    assertTrue(e2.inContactWith(0));
    assertEquals(0, e2.opponentId());
    assertEquals(new Vec2(1, 0), e2.direction());
    // UNKNOWN is explained by #0: L = -4 -> raw 0.0179862; p = 0.0089931, then 0.0134897
    assertEquals(0.0134897, e2.probabilities().get(ContactDetector.UNKNOWN), TOL);
    assertFalse(e2.inContactWith(ContactDetector.UNKNOWN));
    // blocked: L = -4 + 7 = 3 -> raw 0.9525741; p = 0.4762871, then 0.7144306 -> blocked
    assertTrue(e2.blocked());

    // Opponent drives away (gap 1.0) and we stop pushing: raw = sigmoid(-4) = 0.0179862
    // going down: p = raw + (p - raw) * (1 - 0.02 / 0.12) = raw + (p - raw) * 5/6
    // after n steps: 0.0179862 + 0.7318612 * (5/6)^n
    //   n = 5: 0.0179862 + 0.7318612 * 0.4018776 = 0.3121049  (> 0.3: still in contact)
    //   n = 6: 0.0179862 + 0.7318612 * 0.3348980 = 0.2630850  (<= 0.3: out)
    Vec2 far = new Vec2(6.9, 5);
    ContactEstimate e = e2;
    for (int n = 1; n <= 5; n++) {
      double t = 0.04 + 0.02 * n;
      e = d.update(t, DriveTelemetry.idle(t), SELF, List.of(opp(0, far, t)));
    }
    assertEquals(0.3121049, e.probabilities().get(0), TOL);
    assertTrue(e.inContactWith(0)); // hysteresis: below 0.7 but above 0.3
    e = d.update(0.16, DriveTelemetry.idle(0.16), SELF, List.of(opp(0, far, 0.16)));
    assertEquals(0.2630850, e.probabilities().get(0), TOL);
    assertFalse(e.inContact());
    assertEquals(Vec2.ZERO, e.direction());
  }

  @Test
  void blockedWithNobodyTrackedIsUnknownContact() {
    var d = detector();
    // L_unknown = -4 + 7 = 3 -> raw 0.9525741; p = 0, 0.4762871, 0.7144306 -> in contact
    d.update(0.00, stalled(0.00), SELF, List.of());
    d.update(0.02, stalled(0.02), SELF, List.of());
    var e = d.update(0.04, stalled(0.04), SELF, List.of());
    assertEquals(0.7144306, e.probability(), TOL);
    assertTrue(e.inContactWith(ContactDetector.UNKNOWN));
    assertEquals(ContactDetector.UNKNOWN, e.opponentId());
    assertEquals(new Vec2(1, 0), e.direction()); // the push direction
  }

  @Test
  void anOpponentBehindUsDoesNotExplainAForwardStall() {
    var d = detector();
    Vec2 behind = new Vec2(4.1, 5); // gap 0, but cos = (1,0).(-1,0) = -1 < 0 -> does not explain
    // #0: L = -4 + 5.5 = 1.5 -> raw 0.8175745; p = 0, 0.4087872, 0.6131809 (< 0.7)
    // UNKNOWN: L = -4 + 7 = 3 -> raw 0.9525741; p = 0, 0.4762871, 0.7144306 (>= 0.7)
    d.update(0.00, stalled(0.00), SELF, List.of(opp(0, behind, 0.00)));
    d.update(0.02, stalled(0.02), SELF, List.of(opp(0, behind, 0.02)));
    var e = d.update(0.04, stalled(0.04), SELF, List.of(opp(0, behind, 0.04)));
    assertEquals(0.6131809, e.probabilities().get(0), TOL);
    assertFalse(e.inContactWith(0));
    assertTrue(e.inContactWith(ContactDetector.UNKNOWN));
    assertEquals(ContactDetector.UNKNOWN, e.opponentId());
  }

  @Test
  void outputsStayFinite() {
    var d = detector();
    // zero command, zero everything, opponent exactly on top of us
    var e = d.update(0, DriveTelemetry.idle(0), SELF, List.of(opp(0, SELF, 0)));
    e = d.update(0.02, DriveTelemetry.idle(0.02), SELF, List.of(opp(0, SELF, 0.02)));
    assertTrue(Double.isFinite(e.probability()));
    assertTrue(e.direction().isFinite());
    assertTrue(e.pushDirection().isFinite());
  }

  @Test
  void trackWithoutAnIdFeedsUnknownAndNeverCollidesWithIt() {
    var d = detector();
    // A track with id -1 (the same number as UNKNOWN) right in front, gap = 0.9 - 0.9 = 0.
    // Camera only (idle): it gets no hypothesis of its own; its vision goes to UNKNOWN:
    //   L_unknown = -4 + 5.5 * ramp(0; 0.45, 0.05) + 0 = 1.5 -> raw 0.8175745
    //   p: 0 (t = 0, dt = 0) -> 0.4087873 (t = 0.02, factor 0.02/0.04 = 0.5) -> 0.6131809 -> 0.7153777
    Vec2 front = new Vec2(5.9, 5);
    ContactEstimate e = null;
    for (int i = 0; i <= 3; i++) {
      double t = i * 0.02;
      e = d.update(t, DriveTelemetry.idle(t), SELF, List.of(opp(-1, front, t)));
    }
    assertEquals(0.7153777, e.probabilities().get(ContactDetector.UNKNOWN), TOL);
    assertEquals(Set.of(ContactDetector.UNKNOWN), e.probabilities().keySet()); // no "-1 opponent"
    assertTrue(e.inContactWith(ContactDetector.UNKNOWN));
    assertEquals(ContactDetector.UNKNOWN, e.opponentId());
    // direction: toward that track, unit((5.9,5) - (5,5)) = (1,0); not the (zero) push direction
    assertEquals(1.0, e.direction().x(), TOL);
    assertEquals(0.0, e.direction().y(), TOL);
  }

  @Test
  void aRepeatedIdIsNotTwoRobotsUnderOneHypothesis() {
    var d = detector();
    // #7 far away (gap = 3.0 - 0.9 = 2.1, vision 0) and a second "#7" in front (gap 0).
    // The first #7 owns the hypothesis; the repeat counts as an id-less track (-> UNKNOWN), so
    // the far #7 is not stepped twice per loop or credited with the near robot's contact.
    ContactEstimate e = null;
    for (int i = 0; i <= 3; i++) {
      double t = i * 0.02;
      e = d.update(t, DriveTelemetry.idle(t), SELF,
          List.of(opp(7, new Vec2(8, 5), t), opp(7, new Vec2(5.9, 5), t)));
    }
    // #7: L = -4 -> raw 0.0179862; after 3 half-steps p = 0.0179862 * (1 - 0.5^3) = 0.0157379
    assertEquals(0.0157379, e.probabilities().get(7), TOL);
    assertFalse(e.inContactWith(7));
    assertEquals(0.7153777, e.probabilities().get(ContactDetector.UNKNOWN), TOL);
  }

  @Test
  void telemetryFromVelocities() {
    // a = ((1,2) - (0,1)) / 0.02 = (50, 50); no current measured
    var t = DriveTelemetry.fromVelocities(1.0, new Vec2(3, 0), new Vec2(1, 2), new Vec2(0, 1), 0.02);
    assertEquals(50.0, t.measuredAcceleration().x(), TOL);
    assertEquals(50.0, t.measuredAcceleration().y(), TOL);
    assertFalse(t.hasCurrent());
    // dt = 0 -> ZERO, never NaN
    var z = DriveTelemetry.fromVelocities(1.0, Vec2.ZERO, new Vec2(1, 2), Vec2.ZERO, 0.0);
    assertEquals(Vec2.ZERO, z.measuredAcceleration());
  }
}
