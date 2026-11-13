package phantom.core.tracking;

/**
 * Constant-velocity Kalman filter for ONE axis (x or y). Two of these make a 2D track; with
 * independent x/y noise this is exactly the 4-state filter, with simpler math.
 *
 * <p>State [p, v], covariance P = [[P00, P01], [P01, P11]] (symmetric).
 *
 * <pre>
 * Predict by dt (white-noise acceleration with std sigmaA):
 *   p   = p + v dt
 *   P00 = P00 + 2 dt P01 + dt^2 P11 + sigmaA^2 dt^4 / 4
 *   P01 = P01 + dt P11            + sigmaA^2 dt^3 / 2
 *   P11 = P11                     + sigmaA^2 dt^2
 *
 * Update with position measurement z, variance R:
 *   S  = P00 + R,  K0 = P00 / S,  K1 = P01 / S,  y = z - p
 *   p  = p + K0 y,  v = v + K1 y
 *   P00 = (1 - K0) P00,  P01 = (1 - K0) P01,  P11 = P11 - K1 P01(old)
 * </pre>
 */
public final class AxisKalman {
  private double p;
  private double v;
  private double p00;
  private double p01;
  private double p11;

  /**
   * @param position initial position
   * @param velocity initial velocity
   * @param positionVar initial position variance
   * @param velocityVar initial velocity variance
   */
  public AxisKalman(double position, double velocity, double positionVar, double velocityVar) {
    this.p = position;
    this.v = velocity;
    this.p00 = positionVar;
    this.p01 = 0.0;
    this.p11 = velocityVar;
  }

  public void predict(double dt, double sigmaA) {
    double q = sigmaA * sigmaA;
    double dt2 = dt * dt;
    p = p + v * dt;
    double n00 = p00 + 2.0 * dt * p01 + dt2 * p11 + q * dt2 * dt2 / 4.0;
    double n01 = p01 + dt * p11 + q * dt2 * dt / 2.0;
    double n11 = p11 + q * dt2;
    p00 = n00;
    p01 = n01;
    p11 = n11;
  }

  public void update(double z, double measurementVar) {
    double s = p00 + measurementVar;
    double k0 = p00 / s;
    double k1 = p01 / s;
    double y = z - p;
    p = p + k0 * y;
    v = v + k1 * y;
    double oldP01 = p01;
    p00 = (1.0 - k0) * p00;
    p01 = (1.0 - k0) * p01;
    p11 = p11 - k1 * oldP01;
  }

  public double position() {
    return p;
  }

  public double velocity() {
    return v;
  }

  public double positionVar() {
    return p00;
  }

  public double covariance() {
    return p01;
  }

  public double velocityVar() {
    return p11;
  }
}
