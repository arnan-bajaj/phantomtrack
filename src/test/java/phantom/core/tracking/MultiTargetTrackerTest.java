package phantom.core.tracking;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static phantom.core.tracking.AllianceLabel.OPPONENT;
import static phantom.core.tracking.AllianceLabel.PARTNER;
import static phantom.core.tracking.AllianceLabel.UNKNOWN;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import org.junit.jupiter.api.Test;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.MultiTargetTracker.Config;
import phantom.core.tracking.MultiTargetTracker.Stage;
import phantom.core.tracking.MultiTargetTracker.TrackView;

/** MultiTargetTracker: association, life cycle, color, latency, re-acquisition. By hand. */
class MultiTargetTrackerTest {
  private static final double TOL = 1e-9;

  /** Hands the tracker whatever frames the test queued. */
  private static final class Frames implements DetectionSource {
    final List<DetectionFrame> next = new ArrayList<>();

    @Override
    public List<DetectionFrame> poll(double nowSec) {
      List<DetectionFrame> out = new ArrayList<>(next);
      next.clear();
      return out;
    }

    void add(double t, Detection... ds) {
      next.add(new DetectionFrame(t, List.of(ds)));
    }
  }

  private static Detection d(double x, double y, AllianceLabel label) {
    return new Detection(new Vec2(x, y), label);
  }

  /** Feed one frame captured (and delivered) at t, return what the tracker reports at t. */
  private static List<TrackedRobot> frame(
      MultiTargetTracker mtt, Frames q, double t, Detection... ds) {
    q.add(t, ds);
    return mtt.getOpponents(t);
  }

  /** R = std^2, color error 0.1 (one label = ln 9 = 2.1972 of log-odds), no acceleration noise. */
  private static Config still(double std) {
    return Config.defaults(std, 0.1).withMotion(0.0, 1.0);
  }

  // ---------------- association ----------------

  @Test
  void globalNearestNeighborBeatsGreedy() {
    // R = 0.2, accel 0. Tracks A at x=0 and B at x=1 (P00 = R = 0.2, P11 = 4) made at t = 0.
    // At t = 0.1: P00 = 0.2 + 0.1^2 * 4 = 0.24, S = 0.24 + 0.2 = 0.44 on each axis.
    //   d2(A, 0.9) = 0.81 / 0.44 = 1.841    d2(A, 2.0) = 4 / 0.44 = 9.091 (inside 9.210)
    //   d2(B, 0.9) = 0.01 / 0.44 = 0.023    d2(B, 2.0) = 1 / 0.44 = 2.273
    // ln det S is the same for both tracks and UNKNOWN labels add nothing, so compare d2 sums:
    //   A->0.9, B->2.0: 1.841 + 2.273 = 4.114   <-- chosen
    //   A->2.0, B->0.9: 9.091 + 0.023 = 9.114   (what "closest pair first" would pick)
    var q = new Frames();
    var mtt = new MultiTargetTracker(q, still(Math.sqrt(0.2)));
    frame(mtt, q, 0.0, d(0, 0, UNKNOWN), d(1, 0, UNKNOWN));
    frame(mtt, q, 0.1, d(0.9, 0, UNKNOWN), d(2.0, 0, UNKNOWN));
    List<TrackView> t = mtt.tracks();
    assertEquals(2, t.size()); // nothing left over to start a third track
    // K0 = 0.24 / 0.44 = 6/11, K1 = P01 / S = (0.1 * 4) / 0.44 = 10/11
    // A: x = 0 + 6/11 * 0.9 = 5.4/11 = 0.4909,  v = 10/11 * 0.9 = 9/11 = 0.8182
    // B: x = 1 + 6/11 * 1.0 = 17/11 = 1.5455,   v = 10/11 * 1.0 = 0.9091
    assertEquals(1, t.get(0).id());
    assertEquals(5.4 / 11, t.get(0).position().x(), TOL);
    assertEquals(9.0 / 11, t.get(0).velocity().x(), TOL);
    assertEquals(2, t.get(1).id());
    assertEquals(17.0 / 11, t.get(1).position().x(), TOL);
    assertEquals(10.0 / 11, t.get(1).velocity().x(), TOL);
  }

  @Test
  void gateDecidesBetweenUpdatingAndStartingANewTrack() {
    // R = 0.01, accel 0, track at x = 0 made at t = 0. At t = 0.1: P00 = 0.01 + 0.01 * 4 = 0.05,
    // S = 0.06.
    // 0.74 m away: d2 = 0.5476 / 0.06 = 9.127 <= 9.210 -> same track,
    //   x = 0 + (0.05 / 0.06) * 0.74 = 0.6167
    var q = new Frames();
    var mtt = new MultiTargetTracker(q, still(0.1));
    frame(mtt, q, 0.0, d(0, 0, UNKNOWN));
    frame(mtt, q, 0.1, d(0.74, 0, UNKNOWN));
    assertEquals(1, mtt.tracks().size());
    assertEquals(0.74 * 5.0 / 6.0, mtt.tracks().get(0).position().x(), TOL);

    // 0.75 m away: d2 = 0.5625 / 0.06 = 9.375 > 9.210 -> a second track starts at 0.75
    var q2 = new Frames();
    var mtt2 = new MultiTargetTracker(q2, still(0.1));
    frame(mtt2, q2, 0.0, d(0, 0, UNKNOWN));
    frame(mtt2, q2, 0.1, d(0.75, 0, UNKNOWN));
    assertEquals(2, mtt2.tracks().size());
    assertEquals(0.0, mtt2.tracks().get(0).position().x(), TOL);
    assertEquals(0.75, mtt2.tracks().get(1).position().x(), TOL);
  }

  @Test
  void aStaleTrackDoesNotStealFromASharpOne() {
    // R = 0.01, accel 0. B made at t = 0.5 at x = -2.4; A made at t = 0.9 at x = 0.6.
    //   (at 0.9, B: P00 = 0.01 + 0.4^2 * 4 = 0.65, S = 0.66, d2 = 3^2 / 0.66 = 13.6 -> new track)
    // At t = 1.0 one detection at x = 0:
    //   A: S = 0.01 + 0.1^2*4 + 0.01 = 0.06,  d2 = 0.6^2 / 0.06 = 6.000
    //   B: S = 0.01 + 0.5^2*4 + 0.01 = 1.02,  d2 = 2.4^2 / 1.02 = 5.647   (looks closer!)
    //   cost A = 6.000 + ln(0.06 * 0.06) = 6.000 - 5.627 = 0.373   <-- lower, A gets it
    //   cost B = 5.647 + ln(1.02 * 1.02) = 5.647 + 0.040 = 5.687
    assertEquals(0.3731786, MultiTargetTracker.associationCost(6.0, 0.06, 0.06, 1.0), 1e-6);
    assertEquals(5.6866641, MultiTargetTracker.associationCost(5.76 / 1.02, 1.02, 1.02, 1.0), 1e-6);
    var q = new Frames();
    var mtt = new MultiTargetTracker(q, still(0.1));
    frame(mtt, q, 0.5, d(-2.4, 0, UNKNOWN));
    frame(mtt, q, 0.9, d(0.6, 0, UNKNOWN));
    frame(mtt, q, 1.0, d(0.0, 0, UNKNOWN));
    List<TrackView> t = mtt.tracks();
    assertEquals(2, t.size());
    assertEquals(-2.4, t.get(0).position().x(), TOL); // B (id 1) untouched
    // A (id 2): x = 0.6 + (0.05 / 0.06) * (0 - 0.6) = 0.6 - 0.5 = 0.1
    assertEquals(0.1, t.get(1).position().x(), TOL);
  }

  @Test
  void colorLabelDecidesAnOtherwiseEvenMatch() {
    // R = 0.2, e = 0.1, accel 0. At t = 0: O at x = -0.5 labelled OPPONENT -> P(opp) = 0.9,
    // P at x = +0.5 labelled PARTNER -> P(opp) = 0.1. At t = 0.1 one detection at x = 0:
    //   both: S = 0.44, d2 = 0.25 / 0.44 = 0.568, ln det S = 2 ln 0.44 = -1.642
    //   label OPPONENT: P(label | O) = 0.9*0.9 + 0.1*0.1 = 0.82, P(label | P) = 0.1*0.9 + 0.9*0.1 = 0.18
    //   cost O = 0.568 - 1.642 - 2 ln 0.82 = -0.677     cost P = 0.568 - 1.642 - 2 ln 0.18 = 2.356
    assertEquals(-0.6768774,
        MultiTargetTracker.associationCost(0.25 / 0.44, 0.44, 0.44, 0.82), 1e-6);
    assertEquals(2.3558177,
        MultiTargetTracker.associationCost(0.25 / 0.44, 0.44, 0.44, 0.18), 1e-6);

    var q = new Frames();
    var mtt = new MultiTargetTracker(q, still(Math.sqrt(0.2)));
    frame(mtt, q, 0.0, d(-0.5, 0, OPPONENT), d(0.5, 0, PARTNER));
    frame(mtt, q, 0.1, d(0.0, 0, OPPONENT));
    List<TrackView> t = mtt.tracks();
    // O: x = -0.5 + 6/11 * 0.5 = -0.2273, two OPPONENT labels: odds 81:1 -> P = 81/82
    assertEquals(-0.5 + 3.0 / 11, t.get(0).position().x(), TOL);
    assertEquals(81.0 / 82.0, t.get(0).probabilityOpponent(), TOL);
    assertEquals(0.5, t.get(1).position().x(), TOL);

    // Same, but the detection says PARTNER: it goes to P instead (costs swap).
    var q2 = new Frames();
    var mtt2 = new MultiTargetTracker(q2, still(Math.sqrt(0.2)));
    frame(mtt2, q2, 0.0, d(-0.5, 0, OPPONENT), d(0.5, 0, PARTNER));
    frame(mtt2, q2, 0.1, d(0.0, 0, PARTNER));
    t = mtt2.tracks();
    assertEquals(-0.5, t.get(0).position().x(), TOL);
    // P: x = 0.5 - 6/11 * 0.5 = 0.2273, two PARTNER labels: odds 1:81 -> P(opp) = 1/82
    assertEquals(0.5 - 3.0 / 11, t.get(1).position().x(), TOL);
    assertEquals(1.0 / 82.0, t.get(1).probabilityOpponent(), TOL);
  }

  // ---------------- life cycle ----------------

  @Test
  void threeOfFiveThenCoastThenConfirmedAgain() {
    var q = new Frames();
    var mtt = MultiTargetTracker.defaults(q, 0.1, 0.1);
    assertTrue(frame(mtt, q, 0.0, d(5, 5, OPPONENT)).isEmpty()); // hit 1: tentative
    assertTrue(frame(mtt, q, 0.125, d(5, 5, OPPONENT)).isEmpty()); // hit 2: tentative
    List<TrackedRobot> out = frame(mtt, q, 0.25, d(5, 5, OPPONENT)); // hit 3: confirmed
    assertEquals(1, out.size());
    assertEquals(1, out.get(0).id());
    assertEquals(0.25, out.get(0).timestampSec(), TOL);
    // identical detections: the filter never moves, x = 5, v = 0 exactly
    assertEquals(new Vec2(5, 5), out.get(0).position());
    assertEquals(Vec2.ZERO, out.get(0).velocity());
    assertEquals(Stage.CONFIRMED, mtt.tracks().get(0).stage());

    assertEquals(1, frame(mtt, q, 0.375).size()); // empty frame: still reported...
    assertEquals(Stage.COASTING, mtt.tracks().get(0).stage()); // ...but coasting
    frame(mtt, q, 0.5, d(5, 5, OPPONENT));
    assertEquals(Stage.CONFIRMED, mtt.tracks().get(0).stage());
  }

  @Test
  void aTentativeTrackWithTooManyMissesIsDeleted() {
    // 3 of 5: H M M keeps it (2 misses = 5 - 3), the third miss rejects it
    var q = new Frames();
    var mtt = MultiTargetTracker.defaults(q, 0.1, 0.1);
    frame(mtt, q, 0.0, d(5, 5, OPPONENT));
    frame(mtt, q, 0.1);
    frame(mtt, q, 0.2);
    assertEquals(1, mtt.tracks().size());
    assertEquals(Stage.TENTATIVE, mtt.tracks().get(0).stage());
    frame(mtt, q, 0.3);
    assertTrue(mtt.tracks().isEmpty());
  }

  @Test
  void aPartnerIsTrackedButNeverReported() {
    // 5 PARTNER labels at e = 0.1: L = -5 * 2.197 = -10.99, clamped to -10:
    // P(opp) = 1 / (1 + e^10) = 0.0000454
    var q = new Frames();
    var mtt = MultiTargetTracker.defaults(q, 0.1, 0.1);
    for (int i = 0; i < 5; i++) {
      assertTrue(frame(mtt, q, i * 0.1, d(3, 3, PARTNER)).isEmpty());
    }
    TrackView p = mtt.tracks().get(0);
    assertEquals(Stage.CONFIRMED, p.stage());
    assertFalse(p.reported());
    assertEquals(4.5397869e-5, p.probabilityOpponent(), 1e-12);
  }

  @Test
  void timeoutMatchesKalmanOpponentTracker() {
    var q = new Frames();
    var mtt = MultiTargetTracker.defaults(q, 0.1, 0.1);
    for (double t : new double[] {0.0, 0.125, 0.25}) {
      frame(mtt, q, t, d(5, 5, OPPONENT));
    }
    assertEquals(1, mtt.getOpponents(1.25).size()); // age 1.25 - 0.25 = 1.0: kept
    assertTrue(mtt.getOpponents(1.26).isEmpty()); // age 1.01 > 1.0: dropped
    assertTrue(mtt.tracks().isEmpty());
  }

  // ---------------- latency and order ----------------

  @Test
  void lateFramesAreAppliedAtCaptureTimeAndOldFramesDropped() {
    var q = new Frames();
    var mtt = MultiTargetTracker.defaults(q, 0.1, 0.1);
    // three frames arrive together at t = 1.2, out of order; they are used in capture order
    q.add(1.1, d(5, 5, OPPONENT));
    q.add(1.0, d(5, 5, OPPONENT));
    q.add(1.05, d(5, 5, OPPONENT));
    List<TrackedRobot> out = mtt.getOpponents(1.2);
    assertEquals(1, out.size());
    assertEquals(1.1, out.get(0).timestampSec(), TOL); // stamped with the newest capture time
    assertEquals(3, mtt.framesProcessed());
    q.add(1.05, d(9, 9, OPPONENT)); // older than the last processed frame: dropped whole
    q.add(1.1, d(9, 9, OPPONENT)); // same capture time again: dropped too
    mtt.getOpponents(1.3);
    assertEquals(2, mtt.framesDropped());
    assertEquals(1, mtt.tracks().size()); // no track was started at (9, 9)
  }

  @Test
  void oneRobotGivesExactlyTheKnownIdFilter() {
    // Same sightings, 0.1 s late, to both trackers. Once the anonymous track is confirmed it must
    // be the same Kalman filter: same start (P00 = R, P11 = 4), same predict / update, same times.
    var frames = new Frames();
    var known = new ArrayList<Measurement>();
    MeasurementSource knownSource = now -> {
      List<Measurement> out = new ArrayList<>(known);
      known.clear();
      return out;
    };
    var kot = new KalmanOpponentTracker(knownSource, 0.1, 4.0, 1.0);
    var mtt = MultiTargetTracker.defaults(frames, 0.1, 0.05);
    Random rng = new Random(42);
    int compared = 0;
    int nextCapture = 0;
    for (int step = 0; step <= 200; step++) {
      double now = step * 0.02;
      while (nextCapture / 30.0 + 0.1 <= now + 1e-9) {
        double t = nextCapture / 30.0;
        Vec2 z = new Vec2(1 + 2 * t + 0.05 * rng.nextGaussian(), 3 - t + 0.05 * rng.nextGaussian());
        known.add(new Measurement(7, z, t));
        frames.add(t, new Detection(z, OPPONENT));
        nextCapture++;
      }
      List<TrackedRobot> a = kot.getOpponents(now);
      List<TrackedRobot> b = mtt.getOpponents(now);
      if (!b.isEmpty()) {
        assertEquals(1, b.size());
        assertEquals(a.get(0).position().x(), b.get(0).position().x(), 1e-12);
        assertEquals(a.get(0).position().y(), b.get(0).position().y(), 1e-12);
        assertEquals(a.get(0).velocity().x(), b.get(0).velocity().x(), 1e-12);
        assertEquals(a.get(0).velocity().y(), b.get(0).velocity().y(), 1e-12);
        assertEquals(a.get(0).timestampSec(), b.get(0).timestampSec(), 1e-12);
        compared++;
      }
    }
    assertTrue(compared > 180, "compared " + compared);
  }

  // ---------------- confidence ----------------

  @Test
  void confidenceByHand() {
    // P(opp) = 0.9, sigma0 = 0.5 (sigma0^2 = 0.25)
    //   var 0.01: 0.9 * 0.25 / (0.25 + 0.01) = 0.225 / 0.26 = 0.8654
    //   var 1.26: 0.9 * 0.25 / (0.25 + 1.26) = 0.225 / 1.51 = 0.1490
    assertEquals(0.225 / 0.26, MultiTargetTracker.confidence(0.9, 0.01, 0.5), TOL);
    assertEquals(0.225 / 1.51, MultiTargetTracker.confidence(0.9, 1.26, 0.5), TOL);
    assertEquals(1.0, MultiTargetTracker.confidence(1.0, 0.0, 0.5), TOL);
    assertEquals(0.0, MultiTargetTracker.confidence(Double.NaN, 0.01, 0.5), TOL);

    // Through the tracker: confidence drops as the last sighting gets older
    var q = new Frames();
    var mtt = MultiTargetTracker.defaults(q, 0.1, 0.1);
    for (double t : new double[] {0.0, 0.125, 0.25}) {
      frame(mtt, q, t, d(5, 5, OPPONENT));
    }
    double fresh = mtt.getOpponents(0.25).get(0).confidence();
    double old = mtt.getOpponents(0.75).get(0).confidence();
    assertTrue(fresh > old && old > 0.0 && fresh <= 1.0, fresh + " " + old);
  }

  // ---------------- re-acquisition ----------------

  @Test
  void handoffGateByHand() {
    // window 2 s, radius 1.5 m, speed gate 3 m/s, lead 0.5 s. Lost track: p (2,3), v (1,0), t 0.
    Config c = Config.defaults(0.1, 0.1);
    Vec2 p = new Vec2(2, 3);
    Vec2 v = new Vec2(1, 0);
    // t = 1.0: expected = (2,3) + (1,0) * min(1.0, 0.5) = (2.5, 3)
    //   new (3.5, 3) moving (1, 0.5): error |(1, 0)| = 1.0 <= 1.5, speed diff |(0, 0.5)| = 0.5 -> 1.0
    assertEquals(1.0,
        MultiTargetTracker.handoffError(new Vec2(3.5, 3), new Vec2(1, 0.5), 1.0, p, v, 0.0, c), TOL);
    //   new (4.1, 3): error 1.6 > 1.5 -> rejected
    assertEquals(Double.POSITIVE_INFINITY,
        MultiTargetTracker.handoffError(new Vec2(4.1, 3), v, 1.0, p, v, 0.0, c), 0.0);
    //   new moving (-2.5, 0): speed diff |(-3.5, 0)| = 3.5 > 3 -> rejected
    assertEquals(Double.POSITIVE_INFINITY,
        MultiTargetTracker.handoffError(new Vec2(3.5, 3), new Vec2(-2.5, 0), 1.0, p, v, 0.0, c),
        0.0);
    //   2.5 s later (> 2 s window) or before the lost track's time -> rejected
    assertEquals(Double.POSITIVE_INFINITY,
        MultiTargetTracker.handoffError(new Vec2(2.5, 3), v, 2.5, p, v, 0.0, c), 0.0);
    assertEquals(Double.POSITIVE_INFINITY,
        MultiTargetTracker.handoffError(new Vec2(2.5, 3), v, -0.1, p, v, 0.0, c), 0.0);
    // t = 0.2 (< lead): expected = (2,3) + (1,0) * 0.2 = (2.2, 3); new (2.2, 4.4): error 1.4
    assertEquals(1.4,
        MultiTargetTracker.handoffError(new Vec2(2.2, 4.4), v, 0.2, p, v, 0.0, c), TOL);
  }

  /** Confirms a still robot at (5,5) with frames at 0, 0.125, 0.25: track id 1, v = 0. */
  private static void confirmAtFiveFive(MultiTargetTracker mtt, Frames q) {
    for (double t : new double[] {0.0, 0.125, 0.25}) {
      frame(mtt, q, t, d(5, 5, OPPONENT));
    }
    assertEquals(1, mtt.getOpponents(0.25).get(0).id());
  }

  /** The robot is lost (deleted at 1.375 s) and a new track is confirmed at x at t0..t0+0.25. */
  private static List<TrackedRobot> loseThenReappear(Config c, double x, double t0) {
    var q = new Frames();
    var mtt = new MultiTargetTracker(q, c);
    confirmAtFiveFive(mtt, q);
    assertTrue(mtt.getOpponents(1.375).isEmpty()); // age 1.125 > 1.0: gone, id 1 kept as a ghost
    frame(mtt, q, t0, d(x, 5, OPPONENT));
    frame(mtt, q, t0 + 0.125, d(x, 5, OPPONENT));
    return frame(mtt, q, t0 + 0.25, d(x, 5, OPPONENT));
  }

  @Test
  void aLostIdIsTakenOverByATrackBornInTheSamePlace() {
    Config c = Config.defaults(0.1, 0.1);
    // new track confirmed at t = 1.75 at (5.5, 5): dt = 1.75 - 0.25 = 1.5 <= 2,
    // expected = (5,5) + 0 = (5,5), error 0.5 <= 1.5, speed diff 0 -> takes id 1
    List<TrackedRobot> out = loseThenReappear(c, 5.5, 1.5);
    assertEquals(1, out.size());
    assertEquals(1, out.get(0).id());
    // too far: error |(7,5) - (5,5)| = 2.0 > 1.5 -> keeps its own new id 2
    assertEquals(2, loseThenReappear(c, 7.0, 1.5).get(0).id());
    // too late: confirmed at 2.75, ghost expired once 2.5 - 0.25 = 2.25 > 2 -> id 2
    assertEquals(2, loseThenReappear(c, 5.5, 2.5).get(0).id());
    // re-acquisition off -> id 2
    assertEquals(2, loseThenReappear(c.withoutHandoff(), 5.5, 1.5).get(0).id());
  }

  @Test
  void aCoastingCopyIsMergedIntoTheNewTrack() {
    // Track 1 at (5,5) misses from 0.375 on (coasting). A robot shows up at (6.2, 5), outside
    // track 1's gate (at 0.375: S = 0.0364, d2 = 1.2^2 / 0.0364 = 39.6), so it starts track 2,
    // confirmed at 0.625. Track 1 is still coasting (age 0.375 < 1 s):
    //   dt = 0.625 - 0.25 = 0.375, expected = (5,5) + 0, error = 1.2 <= 1.5, speed diff 0
    //   -> track 2 takes id 1 and the coasting copy is deleted.
    var q = new Frames();
    var mtt = MultiTargetTracker.defaults(q, 0.1, 0.1);
    confirmAtFiveFive(mtt, q);
    frame(mtt, q, 0.375, d(6.2, 5, OPPONENT));
    frame(mtt, q, 0.5, d(6.2, 5, OPPONENT));
    List<TrackedRobot> out = frame(mtt, q, 0.625, d(6.2, 5, OPPONENT));
    assertEquals(1, out.size());
    assertEquals(1, out.get(0).id());
    assertEquals(new Vec2(6.2, 5), out.get(0).position());
    assertEquals(1, mtt.tracks().size());
    assertEquals(1, mtt.handoffs());

    // without re-acquisition both are reported: the stale copy (id 1) and the new track (id 2)
    var q2 = new Frames();
    var off = new MultiTargetTracker(q2, Config.defaults(0.1, 0.1).withoutHandoff());
    confirmAtFiveFive(off, q2);
    frame(off, q2, 0.375, d(6.2, 5, OPPONENT));
    frame(off, q2, 0.5, d(6.2, 5, OPPONENT));
    out = frame(off, q2, 0.625, d(6.2, 5, OPPONENT));
    assertEquals(2, out.size());
    assertEquals(new Vec2(5, 5), out.get(0).position());
    assertEquals(2, out.get(1).id());
  }

  @Test
  void aColorFlipReleasesTheIdAndFlippingBackReclaimsIt() {
    // e = 0.1, one label = s = ln 9. Three OPPONENT labels: L = 3s, reported as id 1.
    var q = new Frames();
    var mtt = MultiTargetTracker.defaults(q, 0.1, 0.1);
    confirmAtFiveFive(mtt, q);
    // four PARTNER labels: L = 3s - 4s = -s -> P(opp) = 1/10: not reported, id released
    for (double t : new double[] {0.375, 0.5, 0.625}) {
      frame(mtt, q, t, d(5, 5, PARTNER));
    }
    assertTrue(frame(mtt, q, 0.75, d(5, 5, PARTNER)).isEmpty());
    assertEquals(0.1, mtt.tracks().get(0).probabilityOpponent(), 1e-9);
    assertTrue(mtt.tracks().get(0).id() != 1);
    // two OPPONENT labels: L = -s + 2s = s -> P(opp) = 0.9: reported again, takes id 1 back
    frame(mtt, q, 0.875, d(5, 5, OPPONENT));
    List<TrackedRobot> out = frame(mtt, q, 1.0, d(5, 5, OPPONENT));
    assertEquals(1, out.size());
    assertEquals(1, out.get(0).id());
  }

  // ---------------- robustness ----------------

  @Test
  void badDetectionsAreIgnored() {
    var q = new Frames();
    var mtt = MultiTargetTracker.defaults(q, 0.1, 0.1);
    q.add(0.0, d(Double.NaN, 1, OPPONENT), d(1, Double.POSITIVE_INFINITY, OPPONENT));
    q.add(Double.NaN, d(1, 1, OPPONENT));
    assertTrue(mtt.getOpponents(0.1).isEmpty());
    assertEquals(2, mtt.detectionsIgnored());
    assertEquals(1, mtt.framesDropped());
    assertTrue(mtt.tracks().isEmpty());
  }

  @Test
  void randomGarbageNeverProducesNaN() {
    var q = new Frames();
    var mtt = MultiTargetTracker.defaults(q, 0.25, 0.15);
    Random rng = new Random(7);
    AllianceLabel[] labels = AllianceLabel.values();
    double t = 0.0;
    for (int i = 0; i < 3000; i++) {
      t += rng.nextInt(10) == 0 ? -0.05 : 1.0 / 30.0; // sometimes out of order
      List<Detection> ds = new ArrayList<>();
      int n = rng.nextInt(7);
      for (int k = 0; k < n; k++) {
        double x = rng.nextInt(50) == 0 ? Double.NaN : rng.nextDouble() * 16.5;
        double y = rng.nextDouble() * 8.0;
        ds.add(new Detection(new Vec2(x, y), labels[rng.nextInt(labels.length)]));
      }
      q.next.add(new DetectionFrame(t, ds));
      List<TrackedRobot> out = mtt.getOpponents(t + 0.1);
      Set<Integer> ids = new HashSet<>();
      for (TrackedRobot r : out) {
        assertTrue(r.position().isFinite() && r.velocity().isFinite(), r.toString());
        assertTrue(r.confidence() >= 0.0 && r.confidence() <= 1.0, r.toString());
        assertTrue(Double.isFinite(r.timestampSec()));
        assertTrue(ids.add(r.id()), "duplicate id " + r.id());
      }
    }
    assertTrue(mtt.framesProcessed() > 2000);
  }
}
