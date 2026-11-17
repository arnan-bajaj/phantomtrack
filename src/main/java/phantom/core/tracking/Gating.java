package phantom.core.tracking;

import phantom.core.geometry.Vec2;

/**
 * Mahalanobis gating: is a detection close enough to where a track expects to be seen, measured
 * in units of the track's own uncertainty?
 *
 * <pre>
 *   innovation   y  = z - p_predicted                      (detection minus predicted position)
 *   innovation covariance  S = diag(Sx, Sy),  Sx = P00x_predicted + R,  Sy = P00y_predicted + R
 *   d^2 = y' S^-1 y = y.x^2 / Sx + y.y^2 / Sy              (x and y are independent here)
 *   accept  iff  d^2 <= CHI2_99_2DOF
 * </pre>
 *
 * <p><b>Threshold and source.</b> If the detection really belongs to the track, d^2 follows a
 * chi-square distribution with 2 degrees of freedom (two independent standard normals squared and
 * added). Its 99% quantile is <b>9.210</b>: a true detection lands inside the gate 99 times out
 * of 100. Source: any chi-square table (e.g. NIST/SEMATECH e-Handbook of Statistical Methods,
 * section 1.3.6.7.4, row nu = 2, column 0.99). For 2 degrees of freedom there is also a closed
 * form, P(d^2 <= x) = 1 - exp(-x / 2), which gives x = -2 ln(1 - 0.99) = 9.2103.
 */
public final class Gating {
  private Gating() {}

  /** 99% quantile of the chi-square distribution with 2 degrees of freedom. */
  public static final double CHI2_99_2DOF = 9.210;

  /**
   * Squared Mahalanobis distance for independent axes.
   *
   * @param innovation detection minus predicted position (m)
   * @param varX innovation variance along x (m^2), must be > 0
   * @param varY innovation variance along y (m^2), must be > 0
   * @return d^2 (unitless), or +infinity if a variance is not a positive finite number, so that a
   *     broken covariance can only ever reject a detection and never produce NaN
   */
  public static double mahalanobisSquared(Vec2 innovation, double varX, double varY) {
    if (!(varX > 0.0) || !(varY > 0.0) || !Double.isFinite(varX) || !Double.isFinite(varY)) {
      return Double.POSITIVE_INFINITY;
    }
    double d2 = innovation.x() * innovation.x() / varX + innovation.y() * innovation.y() / varY;
    return Double.isNaN(d2) ? Double.POSITIVE_INFINITY : d2;
  }

  /** True if d^2 is inside the 99% gate (the boundary itself is accepted). */
  public static boolean accepts(double mahalanobisSquared) {
    return mahalanobisSquared <= CHI2_99_2DOF;
  }
}
