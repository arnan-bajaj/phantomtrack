package phantom.core.tracking;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phantom.core.geometry.Vec2;

class GatingTest {
  private static final double TOL = 1e-9;

  @Test
  void thresholdIsTheChiSquare99PercentQuantileFor2Dof() {
    // Closed form for 2 degrees of freedom: P(d^2 <= x) = 1 - exp(-x/2)
    //   x = -2 ln(1 - 0.99) = -2 ln(0.01) = 2 * 4.60517 = 9.21034  -> tables print 9.210
    assertEquals(9.210, Gating.CHI2_99_2DOF, TOL);
    assertEquals(-2.0 * Math.log(0.01), Gating.CHI2_99_2DOF, 5e-4);
  }

  @Test
  void acceptsJustInsideAndRejectsJustOutside() {
    // S = diag(0.25, 0.25)  (0.5 m standard deviation on each axis)
    // inside:  y = (1.5, 0.2): d^2 = 1.5^2/0.25 + 0.2^2/0.25 = 9.00 + 0.16 = 9.16  <= 9.210
    double inside = Gating.mahalanobisSquared(new Vec2(1.5, 0.2), 0.25, 0.25);
    assertEquals(9.16, inside, TOL);
    assertTrue(Gating.accepts(inside));
    // outside: y = (1.5, 0.3): d^2 = 9.00 + 0.3^2/0.25 = 9.00 + 0.36 = 9.36  > 9.210
    double outside = Gating.mahalanobisSquared(new Vec2(1.5, 0.3), 0.25, 0.25);
    assertEquals(9.36, outside, TOL);
    assertFalse(Gating.accepts(outside));
  }

  @Test
  void axesAreWeightedByTheirOwnVariance() {
    // S = diag(0.04, 1.0): the same 0.5 m miss is far in x (sigma 0.2) but close in y (sigma 1)
    // y = (0.5, 0):  d^2 = 0.25 / 0.04 = 6.25
    assertEquals(6.25, Gating.mahalanobisSquared(new Vec2(0.5, 0), 0.04, 1.0), TOL);
    // y = (0, 0.5):  d^2 = 0.25 / 1.0  = 0.25
    assertEquals(0.25, Gating.mahalanobisSquared(new Vec2(0, 0.5), 0.04, 1.0), TOL);
    // y = (0.5, 3):  d^2 = 6.25 + 9/1.0 = 15.25 -> rejected
    assertFalse(Gating.accepts(Gating.mahalanobisSquared(new Vec2(0.5, 3), 0.04, 1.0)));
  }

  @Test
  void boundaryIsAccepted() {
    assertTrue(Gating.accepts(9.210));
    assertFalse(Gating.accepts(9.2101));
    // zero innovation: d^2 = 0
    assertEquals(0.0, Gating.mahalanobisSquared(Vec2.ZERO, 0.01, 0.01), TOL);
  }

  @Test
  void brokenCovarianceRejectsInsteadOfProducingNaN() {
    // 0/0 would be NaN; a NaN must never reach a command, so it becomes "infinitely far".
    assertEquals(Double.POSITIVE_INFINITY, Gating.mahalanobisSquared(Vec2.ZERO, 0.0, 0.25), 0.0);
    assertEquals(Double.POSITIVE_INFINITY, Gating.mahalanobisSquared(new Vec2(1, 1), -1.0, 0.25), 0.0);
    assertEquals(Double.POSITIVE_INFINITY,
        Gating.mahalanobisSquared(new Vec2(1, 1), Double.NaN, 0.25), 0.0);
    assertEquals(Double.POSITIVE_INFINITY,
        Gating.mahalanobisSquared(new Vec2(Double.NaN, 0), 0.25, 0.25), 0.0);
    assertFalse(Gating.accepts(Double.POSITIVE_INFINITY));
    assertFalse(Gating.accepts(Double.NaN));
  }

  @Test
  void axisKalmanPeekMatchesPredictWithoutChangingTheFilter() {
    // p = 1, v = 2, P = [[0.5, 0], [0, 1]]; update z = 2, R = 0.5:
    //   S = 0.5 + 0.5 = 1, K0 = 0.5, K1 = 0, y = 1 -> p = 1.5, v = 2, P00 = 0.25, P01 = 0, P11 = 1
    var k = new AxisKalman(1, 2, 0.5, 1);
    k.update(2, 0.5);
    // peek dt = 0.5, sigmaA = 2:
    //   position = 1.5 + 2 * 0.5 = 2.5
    //   variance = P00 + 2 dt P01 + dt^2 P11 + sigmaA^2 dt^4 / 4
    //            = 0.25 + 0 + 0.25 * 1 + 4 * 0.0625 / 4 = 0.25 + 0.25 + 0.0625 = 0.5625
    assertEquals(2.5, k.predictedPosition(0.5), TOL);
    assertEquals(0.5625, k.predictedPositionVar(0.5, 2), TOL);
    // ... and nothing moved:
    assertEquals(1.5, k.position(), TOL);
    assertEquals(0.25, k.positionVar(), TOL);
    // the real predict lands on exactly the peeked values
    k.predict(0.5, 2);
    assertEquals(2.5, k.position(), TOL);
    assertEquals(0.5625, k.positionVar(), TOL);
  }
}
