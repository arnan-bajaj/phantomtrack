package phantom.core.modes;

import java.util.List;
import java.util.Optional;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Snapshot of everything a defense mode is allowed to look at.
 *
 * @param timeSec current time (s)
 * @param self our pose
 * @param selfVelocity our field velocity (m/s)
 * @param opponents tracked opponents (never null)
 * @param field field geometry
 * @param focusOpponentId opponent the driver selected, or -1 to let the mode decide
 */
public record DefenseContext(
    double timeSec,
    Pose self,
    Vec2 selfVelocity,
    List<TrackedRobot> opponents,
    FieldConfig field,
    int focusOpponentId) {

  /** The driver-selected opponent if it is being tracked, otherwise the nearest one. */
  public Optional<TrackedRobot> focusOpponent() {
    if (focusOpponentId >= 0) {
      for (TrackedRobot r : opponents) {
        if (r.id() == focusOpponentId) {
          return Optional.of(r);
        }
      }
    }
    TrackedRobot best = null;
    double bestDist = Double.POSITIVE_INFINITY;
    for (TrackedRobot r : opponents) {
      double d = r.positionAt(timeSec).distanceTo(self.position());
      if (d < bestDist) {
        bestDist = d;
        best = r;
      }
    }
    return Optional.ofNullable(best);
  }
}
