package phantom.core.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class AnglesTest {
  private static final double TOL = 1e-12;

  @Test
  void wrapKeepsRange() {
    assertEquals(0.0, Angles.wrap(0.0), TOL);
    assertEquals(Math.PI, Angles.wrap(Math.PI), TOL);
    assertEquals(Math.PI, Angles.wrap(-Math.PI), TOL); // (-pi, pi]: -pi maps to +pi
    assertEquals(Math.PI, Angles.wrap(3 * Math.PI), TOL);
    assertEquals(-Math.PI / 2, Angles.wrap(3 * Math.PI / 2), TOL);
    assertEquals(Math.PI / 2, Angles.wrap(-3 * Math.PI / 2), TOL);
    assertEquals(0.5, Angles.wrap(0.5 + 4 * Math.PI), 1e-9);
  }

  @Test
  void differenceTakesShortWayAround() {
    // From 170 deg to -170 deg is +20 deg, not -340 deg.
    assertEquals(Math.toRadians(20), Angles.difference(Math.toRadians(-170), Math.toRadians(170)), 1e-9);
    assertEquals(Math.toRadians(-20), Angles.difference(Math.toRadians(170), Math.toRadians(-170)), 1e-9);
  }
}
