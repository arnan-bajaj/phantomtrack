package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.control.DriverBlend;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/**
 * Seeded randomized invariants for the alliance boundary and the driver's input (plain loops,
 * same numbers every run). Placeholder field 16.54 x 8.07, both symmetries.
 *
 * <ol>
 *   <li>Mirroring red -> blue -> red gives back the original pose, velocity and command.
 *   <li>A blue-frame command moves the blue-frame pose exactly as the real command moves the
 *       real pose (core and the drivetrain agree on where the robot goes).
 *   <li>A red driver's stick always reaches the real field rotated 180 degrees (audit H2).
 *   <li>Driver input above the deadband is never ignored: DriverInput gives a non-zero command,
 *       SafetyGate calls it an override, and the robot's DriverBlend(1, 0) passes it untouched.
 * </ol>
 */
class MirrorAndDriverInvariantTest {
  private static final double L = 16.54;
  private static final double W = 8.07;
  private static final FieldMirror[] MIRRORS = {
    new FieldMirror(L, W, FieldMirror.Symmetry.ROTATED),
    new FieldMirror(L, W, FieldMirror.Symmetry.MIRRORED)
  };

  private static double angle(Random rng) {
    return Angles.wrap((rng.nextDouble() * 2 - 1) * Math.PI);
  }

  @Test
  void redToBlueToRedIsTheIdentity() {
    Random rng = new Random(2026L);
    for (int trial = 0; trial < 100000; trial++) {
      FieldMirror m = MIRRORS[trial % 2];
      // headings exactly at the +-pi edge too
      double h = trial % 50 == 0 ? Math.PI : angle(rng);
      Pose p = new Pose(rng.nextDouble() * L, rng.nextDouble() * W, h);
      Pose back = m.pose(m.pose(p));
      assertEquals(p.position().x(), back.position().x(), 1e-9, p.toString());
      assertEquals(p.position().y(), back.position().y(), 1e-9, p.toString());
      assertTrue(Math.abs(Angles.difference(back.heading(), p.heading())) <= 1e-12, p + " -> " + back);
      assertTrue(back.heading() > -Math.PI && back.heading() <= Math.PI, "out of (-pi, pi]: " + back);
      Vec2 v = new Vec2((rng.nextDouble() * 2 - 1) * 5, (rng.nextDouble() * 2 - 1) * 5);
      Vec2 vBack = m.velocity(m.velocity(v));
      assertEquals(v.x(), vBack.x(), 0.0);
      assertEquals(v.y(), vBack.y(), 0.0);
      ChassisCommand c = new ChassisCommand(v, (rng.nextDouble() * 2 - 1) * 6);
      ChassisCommand cBack = m.command(m.command(c));
      assertEquals(c.velocity(), cBack.velocity());
      assertEquals(c.omega(), cBack.omega(), 0.0);
    }
  }

  @Test
  void aBlueCommandMovesTheMirroredPoseLikeTheRealCommandMovesTheRealPose() {
    Random rng = new Random(11L);
    double dt = 0.02;
    for (int trial = 0; trial < 50000; trial++) {
      FieldMirror m = MIRRORS[trial % 2];
      Pose real = new Pose(rng.nextDouble() * L, rng.nextDouble() * W, angle(rng));
      ChassisCommand blueCmd =
          new ChassisCommand(new Vec2((rng.nextDouble() * 2 - 1) * 4, (rng.nextDouble() * 2 - 1) * 4),
              (rng.nextDouble() * 2 - 1) * 6);
      Pose blue = m.pose(real);
      // what core expects in the blue frame
      Pose blueAfter =
          new Pose(blue.position().plus(blueCmd.velocity().times(dt)),
              Angles.wrap(blue.heading() + blueCmd.omega() * dt));
      // what the real robot does with the converted command, seen in the blue frame
      ChassisCommand realCmd = m.command(blueCmd);
      Pose realAfter =
          new Pose(real.position().plus(realCmd.velocity().times(dt)),
              Angles.wrap(real.heading() + realCmd.omega() * dt));
      Pose seen = m.pose(realAfter);
      assertEquals(blueAfter.position().x(), seen.position().x(), 1e-9);
      assertEquals(blueAfter.position().y(), seen.position().y(), 1e-9);
      assertTrue(Math.abs(Angles.difference(blueAfter.heading(), seen.heading())) <= 1e-9);
    }
  }

  @Test
  void aRedDriversStickAlwaysReachesTheFieldRotated() {
    Random rng = new Random(5L);
    for (int trial = 0; trial < 100000; trial++) {
      FieldMirror m = MIRRORS[trial % 2];
      ChassisCommand stick =
          DriverInput.command(rng.nextDouble() * 2 - 1, rng.nextDouble() * 2 - 1,
              rng.nextDouble() * 2 - 1, 0.1, 1.5, Math.PI);
      ChassisCommand real = m.command(m.driverCommand(stick));
      assertEquals(-stick.velocity().x(), real.velocity().x(), 1e-12);
      assertEquals(-stick.velocity().y(), real.velocity().y(), 1e-12);
      assertEquals(stick.omega(), real.omega(), 1e-12);
    }
  }

  @Test
  void driverInputAboveTheDeadbandIsNeverIgnored() {
    Random rng = new Random(9L);
    SafetyConfig config = SafetyConfig.firstTest(); // deadband 0.1
    SafetyGate gate = new SafetyGate(config,
        new Geofence(7.5, 1.5, 12.5, 6.5, 0.75, 2.0));
    DriverBlend robotBlend = new DriverBlend(1.0, 0.0); // GhostSession's
    int above = 0;
    for (int trial = 0; trial < 100000; trial++) {
      double f = (rng.nextDouble() * 2 - 1) * 0.3;
      double l = (rng.nextDouble() * 2 - 1) * 0.3;
      double r = (rng.nextDouble() * 2 - 1) * 0.3;
      ChassisCommand driver = DriverInput.command(f, l, r, config.stickDeadband(), 1.5, Math.PI);
      double deflection = DriverInput.deflection(f, l, r);
      if (deflection <= config.stickDeadband()) {
        continue;
      }
      above++;
      // non-zero command (translation or rotation)
      assertTrue(driver.velocity().norm() > 0.0 || Math.abs(driver.omega()) > 0.0,
          "deflection " + deflection + " gave STOP");
      // the gate hands control to the driver (trigger held, pose fine, inside the area)
      assertEquals(SafetyGate.Decision.DRIVER_OVERRIDE,
          gate.decide(RobotMode.TELEOP, 1.0, deflection, true, new Vec2(10, 4)));
      // and the blend, with the assist engaged, passes it untouched
      ChassisCommand assist = new ChassisCommand(new Vec2(0.7, -0.2), 1.0);
      assertSame(driver, robotBlend.blend(driver, assist, true, config.speedCap(), config.omegaCap()));
    }
    assertTrue(above > 50000, "too few inputs above the deadband: " + above);
  }
}
