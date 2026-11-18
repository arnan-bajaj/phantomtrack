package phantom.core.tracking;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import phantom.core.geometry.Vec2;

/**
 * Turns noisy, late, gappy sightings into smooth opponent tracks with velocity.
 *
 * <ul>
 *   <li>One constant-velocity Kalman filter per opponent (an {@link AxisKalman} per axis).
 *   <li>Each sighting is applied at its capture time, so camera latency is handled: the filter is
 *       predicted to the capture time, updated, and the returned {@link TrackedRobot} carries that
 *       timestamp. Modes call {@code positionAt(now)} to extrapolate to the present.
 *   <li>Sightings older than the track's latest are ignored (out of order).
 *   <li>Sightings with a NaN or infinite position or time are ignored: a NaN that got into the
 *       filter would stay there for good (and reach the command).
 *   <li>Tracks with no sighting for {@code maxAgeSec} are dropped. Confidence = 1 - age / maxAge.
 * </ul>
 */
public final class KalmanOpponentTracker implements OpponentTracker {
  private final MeasurementSource source;
  private final double measurementStd;
  private final double accelStd;
  private final double maxAgeSec;
  private final Map<Integer, Track> tracks = new HashMap<>();

  private static final class Track {
    final AxisKalman x;
    final AxisKalman y;
    double time;

    Track(AxisKalman x, AxisKalman y, double time) {
      this.x = x;
      this.y = y;
      this.time = time;
    }
  }

  /**
   * @param source where sightings come from
   * @param measurementStd expected position noise of one sighting (m)
   * @param accelStd how hard opponents can change velocity, as a std (m/s^2)
   * @param maxAgeSec drop a track after this long without a sighting (s)
   */
  public KalmanOpponentTracker(
      MeasurementSource source, double measurementStd, double accelStd, double maxAgeSec) {
    this.source = source;
    this.measurementStd = measurementStd;
    this.accelStd = accelStd;
    this.maxAgeSec = maxAgeSec;
  }

  public static KalmanOpponentTracker defaults(MeasurementSource source, double measurementStd) {
    return new KalmanOpponentTracker(source, measurementStd, 4.0, 1.0);
  }

  @Override
  public List<TrackedRobot> getOpponents(double nowSec) {
    List<Measurement> ms = new ArrayList<>(source.poll(nowSec));
    ms.sort(Comparator.comparingDouble(Measurement::timestampSec));
    double r = measurementStd * measurementStd;
    for (Measurement m : ms) {
      if (!m.position().isFinite() || !Double.isFinite(m.timestampSec())) {
        continue; // never let a NaN into a filter
      }
      Track t = tracks.get(m.id());
      if (t == null) {
        // New track: trust the position, assume unknown velocity (std 2 m/s).
        tracks.put(
            m.id(),
            new Track(
                new AxisKalman(m.position().x(), 0.0, r, 4.0),
                new AxisKalman(m.position().y(), 0.0, r, 4.0),
                m.timestampSec()));
        continue;
      }
      double dt = m.timestampSec() - t.time;
      if (dt <= 0) {
        continue; // out of order or duplicate
      }
      t.x.predict(dt, accelStd);
      t.y.predict(dt, accelStd);
      t.x.update(m.position().x(), r);
      t.y.update(m.position().y(), r);
      t.time = m.timestampSec();
    }

    tracks.entrySet().removeIf(e -> nowSec - e.getValue().time > maxAgeSec);

    List<TrackedRobot> out = new ArrayList<>();
    for (Map.Entry<Integer, Track> e : tracks.entrySet()) {
      Track t = e.getValue();
      double age = Math.max(0.0, nowSec - t.time);
      out.add(
          new TrackedRobot(
              e.getKey(),
              new Vec2(t.x.position(), t.y.position()),
              new Vec2(t.x.velocity(), t.y.velocity()),
              t.time,
              Math.max(0.0, 1.0 - age / maxAgeSec)));
    }
    out.sort(Comparator.comparingInt(TrackedRobot::id));
    return out;
  }
}
