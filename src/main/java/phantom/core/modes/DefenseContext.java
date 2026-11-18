package phantom.core.modes;

import java.util.List;
import java.util.Optional;
import java.util.Set;
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
 * @param holdReleased opponents hold ground must not push on (they are driving away; see
 *     {@code phantom.core.manager.HoldLatch}). Empty = hold may push on anyone.
 */
public record DefenseContext(
    double timeSec,
    Pose self,
    Vec2 selfVelocity,
    List<TrackedRobot> opponents,
    FieldConfig field,
    int focusOpponentId,
    Set<Integer> holdReleased) {

  public DefenseContext {
    holdReleased = Set.copyOf(holdReleased);
  }

  /** A context in which hold may push on every opponent. */
  public DefenseContext(double timeSec, Pose self, Vec2 selfVelocity, List<TrackedRobot> opponents,
      FieldConfig field, int focusOpponentId) {
    this(timeSec, self, selfVelocity, opponents, field, focusOpponentId, Set.of());
  }

  /**
   * The driver-selected opponent if one is selected (empty if it is not currently tracked: we
   * search for it rather than chase someone else), otherwise the nearest tracked opponent.
   */
  public Optional<TrackedRobot> focusOpponent() {
    if (focusOpponentId >= 0) {
      for (TrackedRobot r : opponents) {
        if (r.id() == focusOpponentId) {
          return Optional.of(r);
        }
      }
      return Optional.empty();
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
