package phantom.sim;

import java.util.List;
import phantom.core.tracking.OpponentTracker;
import phantom.core.tracking.TrackedRobot;

/** Reports exact ground truth: no noise, no latency, no dropouts. Phase 2 adds a noisy version. */
public final class PerfectTracker implements OpponentTracker {
  private final SimWorld world;

  public PerfectTracker(SimWorld world) {
    this.world = world;
  }

  @Override
  public List<TrackedRobot> getOpponents(double nowSec) {
    return world.truth();
  }
}
