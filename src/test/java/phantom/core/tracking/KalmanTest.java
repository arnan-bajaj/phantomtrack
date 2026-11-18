package phantom.core.tracking;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Test;
import phantom.core.geometry.Vec2;

class KalmanTest {
  private static final double TOL = 1e-9;

  // ---------------- AxisKalman ----------------

  @Test
  void updateThenPredictThenUpdateByHand() {
    // start p=0, v=0, P = [[1,0],[0,1]]
    var k = new AxisKalman(0, 0, 1, 1);

    // update z=2, R=1: S = 1+1 = 2, K0 = 1/2 = 0.5, K1 = 0/2 = 0, y = 2
    // p = 0 + 0.5*2 = 1, v = 0, P00 = 0.5*1 = 0.5, P01 = 0, P11 = 1 - 0*0 = 1
    k.update(2, 1);
    assertEquals(1.0, k.position(), TOL);
    assertEquals(0.0, k.velocity(), TOL);
    assertEquals(0.5, k.positionVar(), TOL);
    assertEquals(0.0, k.covariance(), TOL);
    assertEquals(1.0, k.velocityVar(), TOL);

    // predict dt=1, sigmaA=0: p = 1 + 0*1 = 1
    // P00 = 0.5 + 2*1*0 + 1*1 = 1.5, P01 = 0 + 1*1 = 1, P11 = 1
    k.predict(1, 0);
    assertEquals(1.0, k.position(), TOL);
    assertEquals(1.5, k.positionVar(), TOL);
    assertEquals(1.0, k.covariance(), TOL);
    assertEquals(1.0, k.velocityVar(), TOL);

    // update z=3, R=1: S = 1.5+1 = 2.5, K0 = 1.5/2.5 = 0.6, K1 = 1/2.5 = 0.4, y = 3-1 = 2
    // p = 1 + 0.6*2 = 2.2, v = 0 + 0.4*2 = 0.8
    // P00 = 0.4*1.5 = 0.6, P01 = 0.4*1 = 0.4, P11 = 1 - 0.4*1 = 0.6
    k.update(3, 1);
    assertEquals(2.2, k.position(), TOL);
    assertEquals(0.8, k.velocity(), TOL);
    assertEquals(0.6, k.positionVar(), TOL);
    assertEquals(0.4, k.covariance(), TOL);
    assertEquals(0.6, k.velocityVar(), TOL);
  }

  @Test
  void processNoiseTerms() {
    // P = 0, sigmaA = 2, dt = 0.5: q = 4
    // P00 = 4 * 0.5^4 / 4 = 0.0625, P01 = 4 * 0.5^3 / 2 = 0.25, P11 = 4 * 0.5^2 = 1
    var k = new AxisKalman(0, 0, 0, 0);
    k.predict(0.5, 2);
    assertEquals(0.0625, k.positionVar(), TOL);
    assertEquals(0.25, k.covariance(), TOL);
    assertEquals(1.0, k.velocityVar(), TOL);
  }

  // ---------------- KalmanOpponentTracker ----------------

  /** Hands the tracker whatever the test queued. */
  private static final class Queue implements MeasurementSource {
    final List<Measurement> next = new ArrayList<>();

    @Override
    public List<Measurement> poll(double nowSec) {
      List<Measurement> out = new ArrayList<>(next);
      next.clear();
      return out;
    }
  }

  @Test
  void learnsVelocityFromNoisyMeasurements() {
    // Truth: x = 1 + 2t, y = 3 - 1t. 30 Hz, 0.1 m noise, 3 s.
    var q = new Queue();
    var tracker = new KalmanOpponentTracker(q, 0.1, 4.0, 1.0);
    Random rng = new Random(42);
    List<TrackedRobot> out = List.of();
    for (int i = 0; i <= 90; i++) {
      double t = i / 30.0;
      q.next.add(new Measurement(7, new Vec2(1 + 2 * t + 0.1 * rng.nextGaussian(),
          3 - t + 0.1 * rng.nextGaussian()), t));
      out = tracker.getOpponents(t);
    }
    TrackedRobot r = out.get(0);
    assertEquals(7, r.id());
    assertEquals(2.0, r.velocity().x(), 0.3);
    assertEquals(-1.0, r.velocity().y(), 0.3);
    // at t = 3: truth (7, 0)
    assertEquals(7.0, r.positionAt(3.0).x(), 0.15);
    assertEquals(0.0, r.positionAt(3.0).y(), 0.15);
  }

  @Test
  void lateMeasurementsAreAppliedAtCaptureTime() {
    var q = new Queue();
    var tracker = new KalmanOpponentTracker(q, 0.01, 4.0, 1.0);
    q.next.add(new Measurement(0, new Vec2(5, 5), 1.0));
    // Arrives at t = 1.1 but was captured at 1.0: the track's timestamp must say 1.0
    TrackedRobot r = tracker.getOpponents(1.1).get(0);
    assertEquals(1.0, r.timestampSec(), TOL);
    assertEquals(5.0, r.position().x(), TOL);
  }

  @Test
  void outOfOrderMeasurementsAreIgnored() {
    var q = new Queue();
    var tracker = new KalmanOpponentTracker(q, 0.01, 4.0, 1.0);
    q.next.add(new Measurement(0, new Vec2(5, 5), 1.0));
    tracker.getOpponents(1.0);
    q.next.add(new Measurement(0, new Vec2(9, 9), 0.9)); // older than the track: skip
    TrackedRobot r = tracker.getOpponents(1.1).get(0);
    assertEquals(5.0, r.position().x(), TOL);
    assertEquals(1.0, r.timestampSec(), TOL);
  }

  @Test
  void staleTracksAreDroppedAndConfidenceFades() {
    var q = new Queue();
    var tracker = new KalmanOpponentTracker(q, 0.1, 4.0, 1.0);
    q.next.add(new Measurement(0, new Vec2(5, 5), 0.0));
    // age 0.25 s of max 1.0 -> confidence = 1 - 0.25/1.0 = 0.75
    assertEquals(0.75, tracker.getOpponents(0.25).get(0).confidence(), TOL);
    assertEquals(1, tracker.getOpponents(1.0).size()); // age exactly 1.0: kept
    assertTrue(tracker.getOpponents(1.01).isEmpty()); // older: dropped
  }

  @Test
  void nonFiniteSightingsNeverEnterTheFilter() {
    // Audit H1: one NaN sighting used to make the track NaN for good (and the assist's command
    // with it). A sighting with a NaN or infinite position or time is now dropped, like
    // MultiTargetTracker does.
    var q = new Queue();
    var tracker = new KalmanOpponentTracker(q, 0.01, 4.0, 1.0);
    q.next.add(new Measurement(0, new Vec2(5, 5), 1.0));
    tracker.getOpponents(1.0);
    q.next.add(new Measurement(0, new Vec2(Double.NaN, 5), 1.1));
    q.next.add(new Measurement(0, new Vec2(5, Double.POSITIVE_INFINITY), 1.15));
    q.next.add(new Measurement(0, new Vec2(5, 5), Double.NaN));
    TrackedRobot r = tracker.getOpponents(1.2).get(0);
    // all three dropped: still the sighting at 1.0, untouched
    assertEquals(5.0, r.position().x(), TOL);
    assertEquals(5.0, r.position().y(), TOL);
    assertEquals(0.0, r.velocity().x(), TOL);
    assertEquals(1.0, r.timestampSec(), TOL);
    // a good sighting afterwards is used normally: (5, 5) again at 1.25 keeps x = 5
    q.next.add(new Measurement(0, new Vec2(5, 5), 1.25));
    r = tracker.getOpponents(1.25).get(0);
    assertTrue(r.position().isFinite() && r.velocity().isFinite());
    assertEquals(1.25, r.timestampSec(), TOL);
    // a non-finite first sighting never starts a track
    q.next.add(new Measurement(1, new Vec2(Double.NaN, 1), 1.3));
    assertEquals(1, tracker.getOpponents(1.3).size());
  }
}
