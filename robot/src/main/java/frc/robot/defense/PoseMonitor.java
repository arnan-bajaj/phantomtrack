package frc.robot.defense;

import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;

/**
 * Watches the drivetrain's pose for signs that it cannot be trusted. PhantomTrack does not do
 * localization, so all it can do is refuse to drive on a pose that looks wrong.
 *
 * <pre>
 *   NOT_FINITE     any of x, y, heading is NaN or infinite
 *   STALE          now - poseTimestamp > maxAgeSec           (the estimator stopped updating)
 *   POSITION_JUMP  |p - p_last| > maxJumpMeters in one loop  (e.g. a bad vision correction)
 *   HEADING_JUMP   |wrap(h - h_last)| > maxHeadingJumpRad in one loop (e.g. a flipped tag pose);
 *                  field-relative commands would point the wrong way, so this matters as much
 *   FROZEN         the pose is bit-for-bit identical for frozenSec while we command more than
 *                  FROZEN_MIN_SPEED (a live estimator always moves a little when the robot does)
 *
 *   healthy = no fault for the last recoverySec (and at least recoverySec since the first sample)
 * </pre>
 *
 * Stateful on purpose: it compares each sample with the previous one.
 */
public final class PoseMonitor {
  public enum Fault {
    NONE,
    NOT_FINITE,
    STALE,
    POSITION_JUMP,
    HEADING_JUMP,
    FROZEN
  }

  /** Commanded speed (m/s) above which an unchanging pose counts as frozen. */
  public static final double FROZEN_MIN_SPEED = 0.2;

  private final double maxAgeSec;
  private final double maxJumpMeters;
  private final double maxHeadingJumpRad;
  private final double frozenSec;
  private final double recoverySec;

  private Pose last = null;
  private double lastTime = Double.NaN;
  private double frozenFor = 0.0;
  private double lastBadTime = Double.NaN; // time of the newest fault (or of the first sample)
  private Fault lastFault = Fault.NONE;
  private boolean healthy = false;

  public PoseMonitor(
      double maxAgeSec,
      double maxJumpMeters,
      double maxHeadingJumpRad,
      double frozenSec,
      double recoverySec) {
    this.maxAgeSec = maxAgeSec;
    this.maxJumpMeters = maxJumpMeters;
    this.maxHeadingJumpRad = maxHeadingJumpRad;
    this.frozenSec = frozenSec;
    this.recoverySec = recoverySec;
  }

  public static PoseMonitor of(SafetyConfig c) {
    return new PoseMonitor(
        c.maxPoseAgeSec(),
        c.maxPoseJumpMeters(),
        c.maxHeadingJumpRad(),
        c.frozenPoseSec(),
        c.poseRecoverySec());
  }

  /**
   * Call once per loop.
   *
   * @param nowSec current time (s)
   * @param pose the drivetrain's pose
   * @param poseTimestampSec when that pose was last updated, in the same clock as nowSec
   * @param commandedSpeed the speed we asked the drivetrain for last loop (m/s)
   * @return the fault seen on THIS sample (NONE if it looks fine)
   */
  public Fault update(double nowSec, Pose pose, double poseTimestampSec, double commandedSpeed) {
    Fault fault = Fault.NONE;
    boolean finite =
        pose != null && pose.position().isFinite() && Double.isFinite(pose.heading());
    if (!finite) {
      fault = Fault.NOT_FINITE;
    } else {
      if (!(nowSec - poseTimestampSec <= maxAgeSec)) { // also true for a NaN timestamp
        fault = Fault.STALE;
      }
      if (last != null) {
        double dt = Math.max(0.0, nowSec - lastTime);
        boolean identical =
            pose.position().x() == last.position().x()
                && pose.position().y() == last.position().y()
                && pose.heading() == last.heading();
        frozenFor = identical && commandedSpeed > FROZEN_MIN_SPEED ? frozenFor + dt : 0.0;
        if (pose.position().distanceTo(last.position()) > maxJumpMeters) {
          fault = Fault.POSITION_JUMP;
        } else if (Math.abs(Angles.difference(pose.heading(), last.heading()))
            > maxHeadingJumpRad) {
          fault = Fault.HEADING_JUMP;
        } else if (fault == Fault.NONE && frozenFor >= frozenSec) {
          fault = Fault.FROZEN;
        }
      }
      last = pose;
      lastTime = nowSec;
    }
    if (fault != Fault.NONE) {
      lastFault = fault;
      lastBadTime = nowSec;
    } else if (Double.isNaN(lastBadTime)) {
      lastBadTime = nowSec; // first good sample: start the recovery clock
    }
    healthy = fault == Fault.NONE && nowSec - lastBadTime >= recoverySec;
    return fault;
  }

  /** True once the pose has looked fine for recoverySec. */
  public boolean healthy() {
    return healthy;
  }

  /** The most recent fault ever seen (NONE if there has never been one). For telemetry. */
  public Fault lastFault() {
    return lastFault;
  }
}
