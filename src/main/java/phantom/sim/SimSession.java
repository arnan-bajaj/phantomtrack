package phantom.sim;

import java.util.List;
import java.util.Optional;
import phantom.core.control.ChassisCommand;
import phantom.core.control.DriverBlend;
import phantom.core.control.HolonomicController;
import phantom.core.field.FieldConfig;
import phantom.core.manager.DefenseAssist;
import phantom.core.modes.DefenseTarget;
import phantom.core.modes.InterposeMode;
import phantom.core.modes.LineMirrorMode;
import phantom.core.modes.ShadowMode;
import phantom.core.modes.ZoneMode;
import phantom.core.tracking.KalmanOpponentTracker;
import phantom.core.tracking.OpponentTracker;
import phantom.core.tracking.TrackedRobot;

/**
 * One running simulation: the world, the defense assist wired to a tracker, and the metrics. Used
 * by both the window and headless mode so they behave identically.
 */
public final class SimSession {
  public static final double DT = 0.02; // 50 Hz, same as the robot loop

  /** How the defense code learns where opponents are. */
  public enum Sensor {
    /** Exact ground truth, no delay. */
    PERFECT(null),
    /** Simulated camera + Kalman filter, plausible numbers. */
    REALISTIC(NoisySensor.Config.realistic()),
    /** Simulated camera + Kalman filter, struggling pipeline. */
    BAD(NoisySensor.Config.bad());

    final NoisySensor.Config config;

    Sensor(NoisySensor.Config config) {
      this.config = config;
    }

    public String label() {
      return config == null ? "Perfect" : config.name();
    }

    public Sensor next() {
      return values()[(ordinal() + 1) % values().length];
    }
  }

  private final SimWorld world;
  private final Sensor sensor;
  private final DefenseAssist assist;
  private final Metrics metrics = new Metrics();

  public SimSession(SimWorld world) {
    this(world, Sensor.PERFECT, 1L);
  }

  /**
   * @param seed random seed for the simulated camera (same seed = same run)
   */
  public SimSession(SimWorld world, Sensor sensor, long seed) {
    this.world = world;
    this.sensor = sensor;
    OpponentTracker tracker =
        sensor.config == null
            ? new PerfectTracker(world)
            : KalmanOpponentTracker.defaults(
                new NoisySensor(world, sensor.config, seed), sensor.config.noiseStd());
    this.assist =
        new DefenseAssist(
            List.of(
                ShadowMode.defaults(),
                InterposeMode.defaults(),
                ZoneMode.defaults(),
                LineMirrorMode.defaults()),
            HolonomicController.defaults(),
            DriverBlend.defaults(),
            tracker,
            world.field());
  }

  public static SimSession standard() {
    return standard(Scenarios.OpponentKind.CYCLER);
  }

  public static SimSession standard(Scenarios.OpponentKind kind) {
    return new SimSession(Scenarios.three(FieldConfig.placeholder(), kind));
  }

  public static SimSession standard(Scenarios.OpponentKind kind, Sensor sensor, long seed) {
    return new SimSession(Scenarios.three(FieldConfig.placeholder(), kind), sensor, seed);
  }

  public Sensor sensor() {
    return sensor;
  }

  /** Run one 20 ms loop: assist decides, world moves, metrics update. */
  public void step(ChassisCommand driver, boolean assistEngaged) {
    RobotBody self = world.self();
    ChassisCommand cmd =
        assist.update(world.timeSec(), self.pose(), self.velocity(), driver, assistEngaged);
    world.step(cmd, DT);
    metrics.record(this);
  }

  public SimWorld world() {
    return world;
  }

  public DefenseAssist assist() {
    return assist;
  }

  public Metrics metrics() {
    return metrics;
  }

  /** Running statistics about how well the defense is doing. */
  public static final class Metrics {
    private double engagedSeconds = 0.0;
    private double guardingSeconds = 0.0; // engaged AND reacting to a specific opponent
    private double goalSideSeconds = 0.0;
    private double trackingErrorSum = 0.0;
    private int engagedSamples = 0;
    private double estimateErrorSum = 0.0;
    private int estimateSamples = 0;
    private int opponentSamples = 0;
    private int lastGuardedId = -1;
    private int targetSwitches = 0;

    void record(SimSession s) {
      recordEstimates(s);
      DefenseAssist.State st = s.assist.lastState();
      if (st != DefenseAssist.State.ENGAGED && st != DefenseAssist.State.BACKING_OFF) {
        return;
      }
      Optional<DefenseTarget> target = s.assist.lastTarget();
      if (target.isEmpty()) {
        return;
      }
      engagedSeconds += DT;
      engagedSamples++;
      int guarded = target.get().opponentId();
      if (guarded >= 0) {
        if (lastGuardedId >= 0 && guarded != lastGuardedId) {
          targetSwitches++;
        }
        lastGuardedId = guarded;
      }
      var selfPos = s.world.self().position();
      trackingErrorSum += selfPos.distanceTo(target.get().position());

      // Goal-side = we are closer to the goal than the opponent we are guarding.
      if (target.get().opponentId() < 0) {
        return; // e.g. Zone waiting at its anchor: nobody to be goal-side of
      }
      guardingSeconds += DT;
      var goal = s.world.field().defendedGoal();
      for (TrackedRobot r : s.world.truth()) {
        if (r.id() == target.get().opponentId()
            && selfPos.distanceTo(goal) < r.position().distanceTo(goal)) {
          goalSideSeconds += DT;
        }
      }
    }

    /** How wrong the robot's belief is: |estimated position now - true position|, per track. */
    private void recordEstimates(SimSession s) {
      double now = s.world.timeSec();
      for (TrackedRobot truth : s.world.truth()) {
        opponentSamples++;
        for (TrackedRobot est : s.assist.lastOpponents()) {
          if (est.id() == truth.id()) {
            estimateErrorSum += est.positionAt(now).distanceTo(truth.position());
            estimateSamples++;
          }
        }
      }
    }

    /** Mean distance between estimated and true opponent positions (m), over tracked samples. */
    public double meanEstimateError() {
      return estimateSamples == 0 ? Double.NaN : estimateErrorSum / estimateSamples;
    }

    /** Fraction of the time each opponent was being tracked at all (0..1). */
    public double trackCoverage() {
      return opponentSamples == 0 ? Double.NaN : (double) estimateSamples / opponentSamples;
    }

    /** Times the guarded opponent changed while engaged. */
    public int targetSwitches() {
      return targetSwitches;
    }

    public double engagedSeconds() {
      return engagedSeconds;
    }

    /** Fraction of guarding time spent goal-side of the guarded opponent (0..1). */
    public double goalSideFraction() {
      return guardingSeconds == 0 ? Double.NaN : goalSideSeconds / guardingSeconds;
    }

    /** Average distance between where we are and where the mode wants us (m). */
    public double meanTrackingError() {
      return engagedSamples == 0 ? Double.NaN : trackingErrorSum / engagedSamples;
    }
  }
}
