package phantom.core.tracking;

import phantom.core.geometry.Vec2;

/**
 * One opponent robot as the defense code sees it. In sim this comes from ground truth (plus
 * optional noise); on the real robot it will come from vision.
 *
 * @param id stable identifier for this track
 * @param position field position (m) at {@code timestampSec}
 * @param velocity field velocity (m/s)
 * @param timestampSec time the measurement is valid for (s)
 * @param confidence 0..1; 1 = perfect ground truth
 */
public record TrackedRobot(
    int id, Vec2 position, Vec2 velocity, double timestampSec, double confidence) {

  /** Constant-velocity prediction of where this robot is at {@code timeSec}. */
  public Vec2 positionAt(double timeSec) {
    return position.plus(velocity.times(timeSec - timestampSec));
  }
}
