package phantom.sim;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.Measurement;
import phantom.core.tracking.MeasurementSource;

/**
 * Simulated opponent-detecting camera on our robot. Samples ground truth at a fixed rate and makes
 * it realistically bad: only opponents inside the camera's field of view and range are seen, each
 * sighting can be dropped, positions get Gaussian noise, and results arrive {@code latency} late.
 * Seeded, so runs are repeatable.
 */
public final class NoisySensor implements MeasurementSource {
  /**
   * @param name label for reports
   * @param noiseStd position noise per sighting (m)
   * @param latencySec time from image capture to result (s)
   * @param dropout chance each sighting is lost (0..1)
   * @param fovDeg horizontal field of view, centered on the robot's heading (deg)
   * @param rangeM max detection distance (m)
   * @param rateHz camera frames per second
   */
  public record Config(
      String name,
      double noiseStd,
      double latencySec,
      double dropout,
      double fovDeg,
      double rangeM,
      double rateHz) {

    /**
     * Two forward-ish cameras (140 deg combined) with a decent detector. These numbers are a guess
     * until the real pipeline is measured.
     */
    public static Config realistic() {
      return new Config("Realistic", 0.10, 0.08, 0.15, 140, 8.0, 30);
    }

    /** One 70 deg camera, struggling pipeline: noisier, slower, misses a third of frames. */
    public static Config bad() {
      return new Config("Bad", 0.25, 0.15, 0.35, 70, 6.0, 15);
    }
  }

  private record Pending(Measurement m, double deliverAt) {}

  private final SimWorld world;
  private final Config config;
  private final Random random;
  private final List<Pending> pending = new ArrayList<>();
  private double nextSample = 0.0;

  public NoisySensor(SimWorld world, Config config, long seed) {
    this.world = world;
    this.config = config;
    this.random = new Random(seed);
  }

  public Config config() {
    return config;
  }

  @Override
  public List<Measurement> poll(double nowSec) {
    if (nowSec + 1e-9 >= nextSample) {
      capture(nowSec);
      nextSample = nowSec + 1.0 / config.rateHz();
    }
    List<Measurement> out = new ArrayList<>();
    Iterator<Pending> it = pending.iterator();
    while (it.hasNext()) {
      Pending p = it.next();
      if (p.deliverAt() <= nowSec + 1e-9) {
        out.add(p.m());
        it.remove();
      }
    }
    return out;
  }

  private void capture(double nowSec) {
    RobotBody self = world.self();
    double halfFov = Math.toRadians(config.fovDeg()) / 2.0;
    for (SimWorld.Opponent o : world.opponents()) {
      Vec2 rel = o.body().position().minus(self.position());
      if (rel.norm() > config.rangeM()) {
        continue;
      }
      if (Math.abs(Angles.difference(rel.angle(), self.heading())) > halfFov) {
        continue;
      }
      if (random.nextDouble() < config.dropout()) {
        continue;
      }
      Vec2 noisy =
          o.body()
              .position()
              .plus(new Vec2(random.nextGaussian(), random.nextGaussian()).times(config.noiseStd()));
      pending.add(
          new Pending(new Measurement(o.body().id(), noisy, nowSec), nowSec + config.latencySec()));
    }
  }
}
