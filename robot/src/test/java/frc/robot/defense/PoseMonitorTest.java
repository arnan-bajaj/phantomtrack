package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phantom.core.geometry.Pose;

/**
 * Safety rule: assist disabled when the pose is stale or jumps more than 0.5 m in one loop. Uses
 * SafetyConfig.firstTest(): max age 0.25 s, jump 0.5 m, heading jump 0.5 rad, frozen 0.5 s,
 * recovery 0.5 s. Loop k runs at t = 0.02 k (computed from k, so times don't drift).
 */
class PoseMonitorTest {
  private static PoseMonitor monitor() {
    return PoseMonitor.of(SafetyConfig.firstTest());
  }

  private static double t(int k) {
    return 0.02 * k;
  }

  /** Feeds a robot driving +x at 1 m/s from (10, 4) for loops [from, to]; fresh timestamps. */
  private static void drive(PoseMonitor m, int from, int to) {
    for (int k = from; k <= to; k++) {
      m.update(t(k), new Pose(10.0 + 0.02 * k, 4.0, 0.0), t(k), 1.0);
    }
  }

  @Test
  void healthyOnlyAfterHalfASecondOfGoodSamples() {
    PoseMonitor m = monitor();
    drive(m, 0, 24); // t = 0.48 s since the first sample: 0.48 < 0.5
    assertFalse(m.healthy());
    drive(m, 25, 26); // t = 0.52 s: 0.52 >= 0.5
    assertTrue(m.healthy());
    assertEquals(PoseMonitor.Fault.NONE, m.lastFault());
  }

  @Test
  void aJumpOfMoreThanHalfAMeterInOneLoopIsAFault() {
    PoseMonitor m = monitor();
    drive(m, 0, 50); // x = 10 + 0.02 * 50 = 11.0 at loop 50
    assertTrue(m.healthy());
    // Loop 51: x jumps from 11.0 to 11.51 -> moved 0.51 > 0.5.
    assertEquals(
        PoseMonitor.Fault.POSITION_JUMP, m.update(t(51), new Pose(11.51, 4.0, 0.0), t(51), 1.0));
    assertFalse(m.healthy());
    // It recovers only after 0.5 s of good samples (from the new spot, 0.02 m per loop).
    for (int k = 52; k <= 75; k++) { // t(75) - t(51) = 0.48 s
      m.update(t(k), new Pose(11.51 + 0.02 * (k - 51), 4.0, 0.0), t(k), 1.0);
    }
    assertFalse(m.healthy());
    for (int k = 76; k <= 77; k++) { // 0.52 s
      m.update(t(k), new Pose(11.51 + 0.02 * (k - 51), 4.0, 0.0), t(k), 1.0);
    }
    assertTrue(m.healthy());
    assertEquals(PoseMonitor.Fault.POSITION_JUMP, m.lastFault()); // remembered for telemetry
  }

  @Test
  void aMoveJustUnderTheLimitIsFine() {
    PoseMonitor m = monitor();
    drive(m, 0, 50); // (11.0, 4.0)
    // Diagonal (0.3, 0.39): |d| = sqrt(0.09 + 0.1521) = sqrt(0.2421) = 0.492 < 0.5
    assertEquals(
        PoseMonitor.Fault.NONE, m.update(t(51), new Pose(11.3, 4.39, 0.0), t(51), 1.0));
    assertTrue(m.healthy());
  }

  @Test
  void stalePoseIsAFault() {
    PoseMonitor m = monitor();
    drive(m, 0, 50);
    // Timestamp 0.26 s old (> 0.25).
    assertEquals(
        PoseMonitor.Fault.STALE, m.update(t(51), new Pose(11.02, 4.0, 0.0), t(51) - 0.26, 1.0));
    assertFalse(m.healthy());
    // 0.24 s old is still OK as a sample (but health needs 0.5 s to come back).
    assertEquals(
        PoseMonitor.Fault.NONE, m.update(t(52), new Pose(11.04, 4.0, 0.0), t(52) - 0.24, 1.0));
    // A NaN timestamp counts as stale.
    assertEquals(
        PoseMonitor.Fault.STALE,
        m.update(t(53), new Pose(11.06, 4.0, 0.0), Double.NaN, 1.0));
  }

  @Test
  void headingJumpIsAFaultButWrappingAroundPiIsNot() {
    PoseMonitor m = monitor();
    for (int k = 0; k <= 30; k++) {
      m.update(t(k), new Pose(10.0, 4.0, 3.1), t(k), 0.0);
    }
    // 3.1 -> -3.1: wrap(-3.1 - 3.1) = wrap(-6.2) = -6.2 + 2 pi = 0.0832 rad: a small turn.
    assertEquals(PoseMonitor.Fault.NONE, m.update(t(31), new Pose(10.0, 4.0, -3.1), t(31), 0.0));
    // -3.1 -> -2.5: 0.6 rad in one loop (30 rad/s) > 0.5.
    assertEquals(
        PoseMonitor.Fault.HEADING_JUMP, m.update(t(32), new Pose(10.0, 4.0, -2.5), t(32), 0.0));
    assertFalse(m.healthy());
  }

  @Test
  void identicalPoseWhileCommandingMotionIsFrozen() {
    PoseMonitor m = monitor();
    Pose stuck = new Pose(10.0, 4.0, 0.0);
    m.update(t(0), stuck, t(0), 1.0);
    // Frozen time grows 0.02 s per identical loop: 24 loops = 0.48 s < 0.5.
    for (int k = 1; k <= 24; k++) {
      assertEquals(PoseMonitor.Fault.NONE, m.update(t(k), stuck, t(k), 1.0));
    }
    // 26 loops = 0.52 s >= 0.5.
    m.update(t(25), stuck, t(25), 1.0);
    assertEquals(PoseMonitor.Fault.FROZEN, m.update(t(26), stuck, t(26), 1.0));
  }

  @Test
  void identicalPoseWhileStoppedIsFine() {
    // Commanded speed 0.1 <= FROZEN_MIN_SPEED (0.2): a parked robot's pose may not change.
    PoseMonitor m = monitor();
    Pose parked = new Pose(10.0, 4.0, 0.0);
    for (int k = 0; k <= 100; k++) {
      assertEquals(PoseMonitor.Fault.NONE, m.update(t(k), parked, t(k), 0.1));
    }
    assertTrue(m.healthy());
  }

  @Test
  void nanOrNullPoseIsAFault() {
    PoseMonitor m = monitor();
    drive(m, 0, 50);
    assertEquals(
        PoseMonitor.Fault.NOT_FINITE, m.update(t(51), new Pose(Double.NaN, 4.0, 0.0), t(51), 1.0));
    assertFalse(m.healthy());
    assertEquals(
        PoseMonitor.Fault.NOT_FINITE,
        m.update(t(52), new Pose(11.0, 4.0, Double.POSITIVE_INFINITY), t(52), 1.0));
    assertEquals(PoseMonitor.Fault.NOT_FINITE, m.update(t(53), null, t(53), 1.0));
  }
}
