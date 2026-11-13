package phantom.core.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class InterceptTest {
  private static final double TOL = 1e-9;

  @Test
  void stationaryTarget() {
    // r = (6,0), v = 0, s = 3: a = 0 - 9 = -9, b = 0, c = 36
    // disc = 0 + 4*9*36 = 1296, sqrt = 36, roots = (0 -+ 36) / -18 = 2, -2 -> t = 2
    assertEquals(2.0, Intercept.time(new Vec2(6, 0), Vec2.ZERO, 3.0).getAsDouble(), TOL);
  }

  @Test
  void slowerTargetMovingAway() {
    // r = (6,0), v = (1,0), s = 3: a = 1 - 9 = -8, b = 2*6*1 = 12, c = 36
    // disc = 144 + 4*8*36 = 1296, sqrt = 36, roots = (-12 -+ 36) / -16 = 3, -1.5 -> t = 3
    // check: target at 6 + 1*3 = 9, we travel 3*3 = 9
    assertEquals(3.0, Intercept.time(new Vec2(6, 0), new Vec2(1, 0), 3.0).getAsDouble(), TOL);
  }

  @Test
  void fasterTargetMovingAwayCannotBeCaught() {
    // r = (6,0), v = (5,0), s = 3: a = 25 - 9 = 16, b = 60, c = 36
    // disc = 3600 - 2304 = 1296, roots = (-60 -+ 36) / 32 = -3, -0.75 -> both negative
    assertTrue(Intercept.time(new Vec2(6, 0), new Vec2(5, 0), 3.0).isEmpty());
  }

  @Test
  void fasterTargetComingTowardUs() {
    // r = (6,0), v = (-5,0), s = 3: a = 16, b = -60, c = 36
    // disc = 1296, roots = (60 -+ 36) / 32 = 0.75, 3 -> t = 0.75
    // check: target at 6 - 5*0.75 = 2.25, we travel 3*0.75 = 2.25
    assertEquals(0.75, Intercept.time(new Vec2(6, 0), new Vec2(-5, 0), 3.0).getAsDouble(), TOL);
  }

  @Test
  void sameSpeedClosingUsesLinearCase() {
    // r = (6,0), v = (-3,0), s = 3: a = 0 -> linear, b = 2*6*(-3) = -36, t = -c/b = -36/-36 = 1
    // check: target at 6 - 3 = 3, we travel 3
    assertEquals(1.0, Intercept.time(new Vec2(6, 0), new Vec2(-3, 0), 3.0).getAsDouble(), TOL);
  }

  @Test
  void sameSpeedMovingAwayCannotBeCaught() {
    // a = 0, b = 2*6*3 = 36 > 0 -> no positive root
    assertTrue(Intercept.time(new Vec2(6, 0), new Vec2(3, 0), 3.0).isEmpty());
  }

  @Test
  void crossingTarget() {
    // r = (0,4), v = (3,0), s = 5: a = 9 - 25 = -16, b = 2*(0*3 + 4*0) = 0, c = 16
    // disc = 0 + 4*16*16 = 1024, sqrt = 32, roots = (0 -+ 32) / -32 = 1, -1 -> t = 1
    // check: target at (3,4), distance 5 = 5*1
    assertEquals(1.0, Intercept.time(new Vec2(0, 4), new Vec2(3, 0), 5.0).getAsDouble(), TOL);
  }

  @Test
  void alreadyThere() {
    assertEquals(0.0, Intercept.time(Vec2.ZERO, new Vec2(2, 2), 3.0).getAsDouble(), TOL);
  }

  @Test
  void rejectsZeroSpeed() {
    assertThrows(IllegalArgumentException.class, () -> Intercept.time(new Vec2(1, 0), Vec2.ZERO, 0.0));
  }
}
