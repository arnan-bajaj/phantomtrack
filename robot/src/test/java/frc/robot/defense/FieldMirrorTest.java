package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/**
 * Alliance handling: core always works in the blue frame; on red, poses and commands are
 * mirrored at the boundary. Field 16.54 x 8.07 (the placeholder field).
 */
class FieldMirrorTest {
  private static final double TOL = 1e-9;
  private static final FieldMirror ROTATED =
      new FieldMirror(16.54, 8.07, FieldMirror.Symmetry.ROTATED);
  private static final FieldMirror MIRRORED =
      new FieldMirror(16.54, 8.07, FieldMirror.Symmetry.MIRRORED);

  @Test
  void rotatedPoseOnTheWayIn() {
    // Red robot at (3.0, 2.0), facing 30 deg.
    // x' = 16.54 - 3.0 = 13.54, y' = 8.07 - 2.0 = 6.07, heading' = 30 + 180 = 210 -> -150 deg.
    Pose blue = ROTATED.pose(new Pose(3.0, 2.0, Math.toRadians(30)));
    assertEquals(13.54, blue.position().x(), TOL);
    assertEquals(6.07, blue.position().y(), TOL);
    assertEquals(Math.toRadians(-150), blue.heading(), TOL);
  }

  @Test
  void rotatedVelocityAndCommand() {
    // Measured (1.0, -0.5) m/s -> (-1.0, 0.5). Turning is unchanged by a rotation.
    Vec2 v = ROTATED.velocity(new Vec2(1.0, -0.5));
    assertEquals(-1.0, v.x(), TOL);
    assertEquals(0.5, v.y(), TOL);
    // Core's command (0.8, 0.2), omega 0.5 -> the real field: (-0.8, -0.2), omega 0.5.
    ChassisCommand out = ROTATED.command(new ChassisCommand(new Vec2(0.8, 0.2), 0.5));
    assertEquals(-0.8, out.velocity().x(), TOL);
    assertEquals(-0.2, out.velocity().y(), TOL);
    assertEquals(0.5, out.omega(), TOL);
  }

  @Test
  void mirroredPoseVelocityAndCommand() {
    // (3.0, 2.0, 30 deg) -> x' = 13.54, y' = 2.0 (unchanged), heading' = 180 - 30 = 150 deg.
    Pose blue = MIRRORED.pose(new Pose(3.0, 2.0, Math.toRadians(30)));
    assertEquals(13.54, blue.position().x(), TOL);
    assertEquals(2.0, blue.position().y(), TOL);
    assertEquals(Math.toRadians(150), blue.heading(), TOL);
    // (1.0, -0.5) -> (-1.0, -0.5); a reflection reverses the turning direction: 0.3 -> -0.3.
    Vec2 v = MIRRORED.velocity(new Vec2(1.0, -0.5));
    assertEquals(-1.0, v.x(), TOL);
    assertEquals(-0.5, v.y(), TOL);
    assertEquals(-0.3, MIRRORED.omega(0.3), TOL);
  }

  @Test
  void headingEdgeCases() {
    // Rotated: 0 -> pi, pi -> wrap(2 pi) = 0, -pi/2 (facing -y) -> pi/2.
    assertEquals(Math.PI, ROTATED.heading(0.0), TOL);
    assertEquals(0.0, ROTATED.heading(Math.PI), TOL);
    assertEquals(Math.PI / 2, ROTATED.heading(-Math.PI / 2), TOL);
    // Mirrored: facing +y stays +y (pi - pi/2 = pi/2); 0 -> pi; -pi/2 -> wrap(3 pi / 2) = -pi/2.
    assertEquals(Math.PI / 2, MIRRORED.heading(Math.PI / 2), TOL);
    assertEquals(Math.PI, MIRRORED.heading(0.0), TOL);
    assertEquals(-Math.PI / 2, MIRRORED.heading(-Math.PI / 2), TOL);
  }

  @Test
  void applyingTwiceGivesBackTheOriginal() {
    Pose p = new Pose(3.0, 2.0, Math.toRadians(30));
    for (FieldMirror m : new FieldMirror[] {ROTATED, MIRRORED}) {
      Pose back = m.pose(m.pose(p));
      assertEquals(3.0, back.position().x(), TOL);
      assertEquals(2.0, back.position().y(), TOL);
      assertEquals(Math.toRadians(30), back.heading(), TOL);
      ChassisCommand c = new ChassisCommand(new Vec2(0.8, 0.2), 0.5);
      ChassisCommand cc = m.command(m.command(c));
      assertEquals(0.8, cc.velocity().x(), TOL);
      assertEquals(0.2, cc.velocity().y(), TOL);
      assertEquals(0.5, cc.omega(), TOL);
    }
  }

  @Test
  void aCommandThatMovesTheMirroredPoseMovesTheRealRobotTheSameWay() {
    // The real reason this must be right: core sees the blue-frame pose, sends a blue-frame
    // command, and the real robot must move so that its NEW mirrored pose is where core expected.
    // Red robot at (3.0, 2.0) -> blue (13.54, 6.07). Core commands (-1.0, 0.5) for 0.02 s:
    // expected blue position = (13.54 - 0.02, 6.07 + 0.01) = (13.52, 6.08).
    // Real command = (1.0, -0.5): real position (3.02, 1.99) -> blue (16.54 - 3.02, 8.07 - 1.99)
    // = (13.52, 6.08). Same.
    Vec2 realCmd = ROTATED.velocity(new Vec2(-1.0, 0.5));
    Vec2 realAfter = new Vec2(3.0, 2.0).plus(realCmd.times(0.02));
    Vec2 blueAfter = ROTATED.point(realAfter);
    assertEquals(13.52, blueAfter.x(), TOL);
    assertEquals(6.08, blueAfter.y(), TOL);
    // Same check for heading under MIRRORED: heading 30 deg, core turns at +0.5 rad/s for 0.02 s.
    // Blue heading 150 deg + 0.01 rad. Real omega = -0.5: real heading 30 deg - 0.01 rad
    // -> mirrored 180 deg - (30 deg - 0.01 rad) = 150 deg + 0.01 rad. Same.
    double realHeading = Math.toRadians(30) + MIRRORED.omega(0.5) * 0.02;
    assertEquals(Math.toRadians(150) + 0.01, MIRRORED.heading(realHeading), TOL);
  }

  @Test
  void aRedDriversStickIsRotatedNotReflected() {
    // Audit H2. A red driver stands at the red wall facing -x, whatever the field's symmetry: their
    // forward is real -x, their left is real -y, and a CCW twist is real CCW. DriverInput builds the
    // stick command as if standing at the blue wall: stick forward 0.8 m/s, left 1.0 m/s, CCW
    // 0.5 rad/s -> (0.8, 1.0), omega 0.5. The robot must get the 180 deg rotation of that:
    // real (-0.8, -1.0), omega 0.5.
    ChassisCommand stick = new ChassisCommand(new Vec2(0.8, 1.0), 0.5);
    for (FieldMirror m : new FieldMirror[] {ROTATED, MIRRORED}) {
      // DefenseTeleop's path: stick -> blue frame (driverCommand) -> PhantomTrack hands it back
      // untouched while the driver drives -> real field (command)
      ChassisCommand real = m.command(m.driverCommand(stick));
      assertEquals(-0.8, real.velocity().x(), TOL, m.symmetry().toString());
      assertEquals(-1.0, real.velocity().y(), TOL, m.symmetry().toString());
      assertEquals(0.5, real.omega(), TOL, m.symmetry().toString());
    }
    // In the blue frame (what core and the ghosts see):
    //   ROTATED:  mirror(rotate(c)) = rotate(rotate(c)) = c            -> (0.8, 1.0), 0.5
    //   MIRRORED: rotate (-0.8, -1.0, 0.5), then reflect (-vx, vy, -w) -> (0.8, -1.0), -0.5
    ChassisCommand rotatedBlue = ROTATED.driverCommand(stick);
    assertEquals(0.8, rotatedBlue.velocity().x(), TOL);
    assertEquals(1.0, rotatedBlue.velocity().y(), TOL);
    assertEquals(0.5, rotatedBlue.omega(), TOL);
    ChassisCommand mirroredBlue = MIRRORED.driverCommand(stick);
    assertEquals(0.8, mirroredBlue.velocity().x(), TOL);
    assertEquals(-1.0, mirroredBlue.velocity().y(), TOL);
    assertEquals(-0.5, mirroredBlue.omega(), TOL);
  }

  @Test
  void theRedGoalIsTheBlueGoalRotated() {
    // Core defends (13.0, 4.035) in the blue frame. As red, that is (16.54 - 13.0, 8.07 - 4.035)
    // = (3.54, 4.035) on the real field: the other end, which is what we want.
    Vec2 real = ROTATED.point(new Vec2(13.0, 4.035));
    assertEquals(3.54, real.x(), TOL);
    assertEquals(4.035, real.y(), TOL);
  }
}
