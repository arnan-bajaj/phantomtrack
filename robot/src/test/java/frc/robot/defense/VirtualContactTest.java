package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.geometry.Vec2;

/**
 * Ghosts push back. Robot half size 0.45 (touching at 0.90 m, counted as contact up to 0.95 m),
 * speed cap 1.0 m/s. Contact.PUSH_SPEED = 2.0 is scaled by cap / 4.5.
 */
class VirtualContactTest {
  private static final double TOL = 1e-9;
  private static final double HALF = 0.45;

  private static VirtualContact.Ghost parked(double x, double y) {
    return new VirtualContact.Ghost(new Vec2(x, y), Vec2.ZERO, 4.5);
  }

  private static Vec2 limit(Vec2 cmd, List<VirtualContact.Ghost> ghosts) {
    return VirtualContact.limit(cmd, Vec2.ZERO, ghosts, HALF, 1.0);
  }

  @Test
  void ghostNotTouchingChangesNothing() {
    // Gap = 1.0 - 0.9 = 0.10 > 0.05.
    Vec2 out = limit(new Vec2(1, 0), List.of(parked(1.0, 0)));
    assertEquals(1.0, out.x(), TOL);
  }

  @Test
  void pushingAParkedGhostIsSlowedToThePushingSpeed() {
    // n = (1, 0); e_self = (1, 0).(1, 0) / 1.0 = 1; e_ghost = 0.
    // pair = 2.0 * (1 - 0) / 2 * 1.0 / 4.5 = 0.2222222
    Vec2 out = limit(new Vec2(1, 0), List.of(parked(0.9, 0)));
    assertEquals(1.0 / 4.5, out.x(), TOL);
    assertEquals(0.0, out.y(), TOL);
  }

  @Test
  void equalPushIsAStall() {
    // Ghost drives into us flat out: desired (-4.5, 0), e_ghost = (-4.5, 0).(-1, 0) / 4.5 = 1.
    // pair = 2.0 * (1 - 1) / 2 = 0 -> nothing into the ghost.
    var ghost = new VirtualContact.Ghost(new Vec2(0.9, 0), new Vec2(-4.5, 0), 4.5);
    Vec2 out = limit(new Vec2(1, 0), List.of(ghost));
    assertEquals(0.0, out.x(), TOL);
  }

  @Test
  void aStrongerGhostPushesTheAssistBack() {
    // Our command (0.5, 0): e_self = 0.5. Ghost full: e_ghost = 1.
    // pair = 2.0 * (0.5 - 1) / 2 * 1/4.5 = -0.5 / 4.5 = -0.1111111 -> we give way at 0.111 m/s.
    var ghost = new VirtualContact.Ghost(new Vec2(0.9, 0), new Vec2(-4.5, 0), 4.5);
    Vec2 out = limit(new Vec2(0.5, 0), List.of(ghost));
    assertEquals(-0.5 / 4.5, out.x(), TOL);
    assertEquals(0.0, out.y(), TOL);
  }

  @Test
  void aGhostPushingAParkedAssistMovesItAtThePushSpeed() {
    // Assist wants to stay put (0, 0): e_self = 0. Ghost full: e_ghost = 1.
    // pair = 2.0 * (0 - 1) / 2 / 4.5 = -0.2222222; into = 0 > -0.2222 -> v = (-0.2222222, 0).
    // Same as the sim: a robot pushed by a flat-out opponent slides at PUSH_SPEED / 2 of 4.5.
    var ghost = new VirtualContact.Ghost(new Vec2(0.9, 0), new Vec2(-4.5, 0), 4.5);
    Vec2 out = limit(Vec2.ZERO, List.of(ghost));
    assertEquals(-1.0 / 4.5, out.x(), TOL);
  }

  @Test
  void backingAwayFasterThanThePushIsUntouched() {
    // Ghost full: pair = -0.2222 (with our effort -1: 2.0 * (-1 - 1) / 2 / 4.5 = -0.4444).
    // Our command (-1, 0): into = -1 < -0.4444 -> unchanged.
    var ghost = new VirtualContact.Ghost(new Vec2(0.9, 0), new Vec2(-4.5, 0), 4.5);
    Vec2 out = limit(new Vec2(-1.0, 0), List.of(ghost));
    assertEquals(-1.0, out.x(), TOL);
  }

  @Test
  void slidingAlongAGhostIsKept() {
    // Command (0.6, 0.8): e_self = 0.6 -> pair = 2.0 * 0.6 / 2 / 4.5 = 0.1333333.
    // into = 0.6 > 0.1333 -> v = (0.6, 0.8) - (0.6 - 0.1333333) * (1, 0) = (0.1333333, 0.8).
    Vec2 out = limit(new Vec2(0.6, 0.8), List.of(parked(0.9, 0)));
    assertEquals(0.6 / 4.5, out.x(), TOL);
    assertEquals(0.8, out.y(), TOL);
  }

  @Test
  void drivingAwayIsUntouched() {
    Vec2 out = limit(new Vec2(-1, 0.2), List.of(parked(0.9, 0)));
    assertEquals(-1.0, out.x(), TOL);
    assertEquals(0.2, out.y(), TOL);
  }

  @Test
  void twoGhostsInACorner() {
    // Ghosts at (0.9, 0) and (0, 0.9), both parked. Command (0.5, 0.5).
    // Ghost A: n = (1, 0), e_self = 0.5 -> allow 2.0 * 0.5 / 2 / 4.5 = 0.1111111 -> v = (0.1111, 0.5)
    // Ghost B: n = (0, 1), e_self from the ORIGINAL command = 0.5 -> allow 0.1111
    //          -> v = (0.1111, 0.1111)
    Vec2 out = limit(new Vec2(0.5, 0.5), List.of(parked(0.9, 0), parked(0, 0.9)));
    assertEquals(0.5 / 4.5, out.x(), TOL);
    assertEquals(0.5 / 4.5, out.y(), TOL);
  }

  @Test
  void coincidentCentersAndBadInputAreSafe() {
    // Centers on top of each other: n = (1, 0) as in the sim. (1, 0) -> limited to 1/4.5.
    Vec2 same = limit(new Vec2(1, 0), List.of(parked(0, 0)));
    assertEquals(1.0 / 4.5, same.x(), TOL);
    assertEquals(Vec2.ZERO, limit(new Vec2(Double.NaN, 0), List.of(parked(0.9, 0))));
    assertEquals(
        Vec2.ZERO,
        VirtualContact.limit(new Vec2(1, 0), new Vec2(Double.NaN, 0), List.of(), HALF, 1.0));
    assertEquals(Vec2.ZERO, VirtualContact.limit(new Vec2(1, 0), Vec2.ZERO, List.of(), HALF, 0.0));
  }
}
