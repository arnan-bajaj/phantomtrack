package phantom.core.geometry;

/** Angle helpers. All angles are radians. */
public final class Angles {
  private Angles() {}

  /** Wraps an angle into (-pi, pi]. */
  public static double wrap(double angleRad) {
    double a = Math.IEEEremainder(angleRad, 2.0 * Math.PI); // result in [-pi, pi]
    return a <= -Math.PI ? a + 2.0 * Math.PI : a;
  }

  /** Shortest signed rotation from {@code from} to {@code to}, in (-pi, pi]. */
  public static double difference(double to, double from) {
    return wrap(to - from);
  }
}
