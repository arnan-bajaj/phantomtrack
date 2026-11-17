package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;

/** Stick shaping and the override measure. Deadband 0.1, 1.5 m/s, pi rad/s. */
class DriverInputTest {
  private static final double TOL = 1e-9;

  private static ChassisCommand cmd(double f, double l, double r) {
    return DriverInput.command(f, l, r, 0.1, 1.5, Math.PI);
  }

  @Test
  void radialDeadbandAndScaling() {
    // 0.55 forward: (0.55 - 0.1) / 0.9 = 0.5 -> 0.5 * 1.5 = 0.75 m/s along +x.
    ChassisCommand a = cmd(0.55, 0.0, 0.0);
    assertEquals(0.75, a.velocity().x(), TOL);
    assertEquals(0.0, a.velocity().y(), TOL);
    // (0.6, 0.8): m = 1 -> full speed in that direction: (0.6, 0.8) * 1.5 = (0.9, 1.2).
    ChassisCommand b = cmd(0.6, 0.8, 0.0);
    assertEquals(0.9, b.velocity().x(), TOL);
    assertEquals(1.2, b.velocity().y(), TOL);
    // (1, 1): m = 1.4142 is clamped to 1 -> (1, 1) / 1.4142 * 1.5 = (1.0606602, 1.0606602).
    ChassisCommand c = cmd(1.0, 1.0, 0.0);
    assertEquals(1.5 / Math.sqrt(2.0), c.velocity().x(), TOL);
    assertEquals(1.5, c.velocity().norm(), TOL);
    // (0.06, 0.08): m = 0.1 <= 0.1 -> nothing.
    assertEquals(0.0, cmd(0.06, 0.08, 0.0).velocity().norm(), TOL);
  }

  @Test
  void rotationDeadbandAndScaling() {
    // 0.55 -> 0.5 * pi = 1.5707963; -1 -> -pi; 0.1 -> 0.
    assertEquals(Math.PI / 2, cmd(0, 0, 0.55).omega(), TOL);
    assertEquals(-Math.PI, cmd(0, 0, -1.0).omega(), TOL);
    assertEquals(0.0, cmd(0, 0, 0.1).omega(), TOL);
  }

  @Test
  void deflectionIsTheLargestRawInput() {
    // max(|(0.06, 0.08)| = 0.1, |-0.3|) = 0.3
    assertEquals(0.3, DriverInput.deflection(0.06, 0.08, -0.3), TOL);
    assertEquals(0.5, DriverInput.deflection(0.3, 0.4, 0.0), TOL); // 3-4-5
  }

  @Test
  void brokenControllerGivesNoCommandButCountsAsOverride() {
    ChassisCommand c = cmd(Double.NaN, 0.5, Double.NaN);
    assertEquals(0.0, c.velocity().norm(), TOL);
    assertEquals(0.0, c.omega(), TOL);
    assertTrue(Double.isNaN(DriverInput.deflection(Double.NaN, 0.0, 0.0)));
    // SafetyGate treats NaN deflection as DRIVER_OVERRIDE (see SafetyGateTest).
  }
}
