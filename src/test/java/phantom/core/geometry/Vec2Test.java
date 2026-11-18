package phantom.core.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class Vec2Test {
  private static final double TOL = 1e-12;

  @Test
  void basicArithmetic() {
    Vec2 a = new Vec2(3, 4);
    Vec2 b = new Vec2(-1, 2);
    assertEquals(new Vec2(2, 6), a.plus(b));
    assertEquals(new Vec2(4, 2), a.minus(b));
    assertEquals(new Vec2(6, 8), a.times(2));
    assertEquals(5.0, a.dot(b), TOL); // 3*(-1) + 4*2 = 5
    assertEquals(10.0, a.cross(b), TOL); // 3*2 - 4*(-1) = 10
    assertEquals(5.0, a.norm(), TOL);
    assertEquals(25.0, a.normSquared(), TOL);
  }

  @Test
  void normalizedHasLengthOne() {
    Vec2 u = new Vec2(3, 4).normalized();
    assertEquals(0.6, u.x(), TOL);
    assertEquals(0.8, u.y(), TOL);
    assertEquals(1.0, u.norm(), TOL);
  }

  @Test
  void normalizedZeroIsZeroNotNaN() {
    Vec2 u = Vec2.ZERO.normalized();
    assertEquals(Vec2.ZERO, u);
    assertTrue(u.isFinite());
  }

  @Test
  void clampNormOnlyShrinks() {
    assertEquals(new Vec2(3, 4), new Vec2(3, 4).clampNorm(10));
    Vec2 c = new Vec2(30, 40).clampNorm(5);
    assertEquals(3.0, c.x(), TOL);
    assertEquals(4.0, c.y(), TOL);
  }

  @Test
  void angleAndPolarRoundTrip() {
    Vec2 v = Vec2.polar(2.0, Math.toRadians(135));
    assertEquals(-Math.sqrt(2), v.x(), TOL);
    assertEquals(Math.sqrt(2), v.y(), TOL);
    assertEquals(Math.toRadians(135), v.angle(), TOL);
    assertEquals(0.0, Vec2.ZERO.angle(), TOL);
  }

  @Test
  void angleStaysInTheHalfOpenRangeForNegativeZero() {
    // Audit L5. atan2(-0.0, -1) = -pi, outside (-pi, pi]. (1,0).times(-1) = (-1, -0.0) is the
    // usual way to get there. Expected pi, exactly as for (-1, +0.0).
    assertEquals(Math.PI, new Vec2(1, 0).times(-1.0).angle(), 0.0);
    assertEquals(Math.PI, new Vec2(-1, -0.0).angle(), 0.0);
    assertEquals(Math.PI, new Vec2(-1, 0.0).angle(), 0.0);
    // nothing else moves: (0,-1) -> -pi/2, (1,-0.0) -> -0.0 == 0
    assertEquals(-Math.PI / 2, new Vec2(0, -1).angle(), 0.0);
    assertEquals(0.0, new Vec2(1, -0.0).angle(), 0.0);
  }

  @Test
  void distanceAndLerp() {
    assertEquals(5.0, new Vec2(1, 1).distanceTo(new Vec2(4, 5)), TOL);
    assertEquals(new Vec2(2, 3), new Vec2(0, 0).lerp(new Vec2(4, 6), 0.5));
  }
}
