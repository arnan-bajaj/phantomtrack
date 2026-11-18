package phantom.sim;

import java.util.Random;
import phantom.core.geometry.Vec2;
import phantom.core.rules.DriveTelemetry;

/**
 * Simulated drive sensors on one robot: what the swerve modules were asked to do, what the wheels
 * and IMU measured, and how much current the drive motors pulled. Samples at a fixed period (20 ms
 * by default, the robot loop) and adds Gaussian noise. Seeded, so runs are repeatable.
 *
 * <pre>
 *   module i sits at r_i = rotate((+-m, +-m), heading)            (m = moduleOffsetM)
 *   module velocity       v_i     = v     + omega     * perp(r_i)     perp(x, y) = (-y, x)
 *   commanded             v_cmd_i = v_cmd + omega_cmd * perp(r_i)
 *   wheel speeds          s_cmd_i = |v_cmd_i|,   s_i = v_i . unit(v_cmd_i)
 *                         (if the module is asked to stop: s_cmd_i = 0, s_i = |v_i|)
 *   I_i = DriveMotorModel.moduleCurrent(s_cmd_i, s_i) + noise
 *   driveCurrentAmps = mean of |I_i| over the four modules
 * </pre>
 *
 * The sim's motion is kinematic (speed and acceleration limits, not forces), so the current is an
 * estimate that follows the speed error, not a physics result.
 */
public final class DriveTelemetrySim {
  /**
   * @param samplePeriodSec time between samples (s)
   * @param velocityNoiseStd noise on each axis of the measured velocity (m/s)
   * @param accelNoiseStd noise on each axis of the measured acceleration (m/s^2)
   * @param currentNoiseStd noise on each module's current (A)
   */
  public record Config(
      double samplePeriodSec, double velocityNoiseStd, double accelNoiseStd, double currentNoiseStd) {

    /**
     * 20 ms samples; wheel odometry good to 0.03 m/s, a filtered IMU to 0.5 m/s^2, current to 2 A.
     * PLACEHOLDERS until measured on the real robot.
     */
    public static Config realistic() {
      return new Config(0.02, 0.03, 0.5, 2.0);
    }

    /** 20 ms samples, no noise. */
    public static Config noiseless() {
      return new Config(0.02, 0.0, 0.0, 0.0);
    }
  }

  private final RobotBody body;
  private final DriveMotorModel model;
  private final Config config;
  private final Random random;
  private double nextSample = Double.NEGATIVE_INFINITY;
  private DriveTelemetry last = null;
  private double[] lastModuleCurrents = new double[4];

  public DriveTelemetrySim(RobotBody body, DriveMotorModel model, Config config, long seed) {
    this.body = body;
    this.model = model;
    this.config = config;
    this.random = new Random(seed);
  }

  /** The newest sample at {@code nowSec}; a new one is taken every sample period. */
  public DriveTelemetry sample(double nowSec) {
    if (last == null || nowSec + 1e-9 >= nextSample) {
      last = measure(nowSec);
      nextSample = nowSec + config.samplePeriodSec();
    }
    return last;
  }

  private DriveTelemetry measure(double nowSec) {
    double[] currents =
        moduleCurrents(
            model,
            body.desiredVelocity(),
            body.desiredOmega(),
            body.velocity(),
            body.omega(),
            body.heading());
    double sum = 0.0;
    for (int i = 0; i < currents.length; i++) {
      currents[i] += random.nextGaussian() * config.currentNoiseStd();
      sum += Math.abs(currents[i]);
    }
    lastModuleCurrents = currents;
    Vec2 v = body.velocity().plus(noise(config.velocityNoiseStd()));
    Vec2 a = body.acceleration().plus(noise(config.accelNoiseStd()));
    return new DriveTelemetry(nowSec, body.desiredVelocity(), v, a, sum / currents.length);
  }

  private Vec2 noise(double std) {
    return new Vec2(random.nextGaussian() * std, random.nextGaussian() * std);
  }

  /** Signed current of each module at the last sample (A), noise included. For logging. */
  public double[] lastModuleCurrents() {
    return lastModuleCurrents.clone();
  }

  /**
   * Noise-free current of the four modules (A), in the order front-left, front-right, rear-left,
   * rear-right. See the class comment for the formula.
   */
  public static double[] moduleCurrents(
      DriveMotorModel model,
      Vec2 commandedVelocity,
      double commandedOmega,
      Vec2 velocity,
      double omega,
      double heading) {
    double m = model.moduleOffsetM();
    double[][] corners = {{m, m}, {m, -m}, {-m, m}, {-m, -m}};
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    double[] out = new double[corners.length];
    for (int i = 0; i < corners.length; i++) {
      double rx = corners[i][0] * cos - corners[i][1] * sin;
      double ry = corners[i][0] * sin + corners[i][1] * cos;
      Vec2 perp = new Vec2(-ry, rx);
      Vec2 vCmd = commandedVelocity.plus(perp.times(commandedOmega));
      Vec2 v = velocity.plus(perp.times(omega));
      double sCmd = vCmd.norm();
      double s = sCmd < Vec2.EPS ? v.norm() : v.dot(vCmd.normalized());
      out[i] = model.moduleCurrent(sCmd < Vec2.EPS ? 0.0 : sCmd, s);
    }
    return out;
  }
}
