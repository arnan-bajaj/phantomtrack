package frc.robot.defense;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import phantom.core.control.DriverBlend;
import phantom.core.control.HolonomicController;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.manager.DefenseAssist;
import phantom.core.modes.CutoffMode;
import phantom.core.modes.LineMirrorMode;
import phantom.core.modes.ShadowMode;
import phantom.core.modes.ZoneMode;
import phantom.core.rules.PinRules;
import phantom.core.rules.PinTimer;
import phantom.sim.CyclerBehavior;
import phantom.sim.PerfectTracker;
import phantom.sim.RobotBody;
import phantom.sim.Scenarios;
import phantom.sim.SimWorld;

/**
 * One running set of ghost opponents plus the defense assist that guards against them: the robot's
 * version of the simulator's SimSession. It reuses the simulator's own classes (CyclerBehavior,
 * Avoidance, Contact, SimWorld), so a ghost behaves exactly like a sim opponent. The one
 * difference: our robot is real ({@link SimWorld#stepWithRealSelf}), so ghosts react to its
 * measured pose and can never move it.
 *
 * <p>A session is thrown away and rebuilt whenever a setting changes; ghosts then restart from
 * their pickup spots. Its clock is the ghost world's own time, which only advances while ghosts
 * are stepped.
 */
public final class GhostSession {
  /** Our robot's id inside the ghost world (the same number the simulator uses). */
  public static final int SELF_ID = 100;

  /** Mode list positions, in D-pad order: up, right, down, left. */
  public static final int MODE_LINE_MIRROR = 0;

  public static final int MODE_CUTOFF = 1;
  public static final int MODE_SHADOW = 2;
  public static final int MODE_ZONE = 3;

  /** Ghosts score from this far out and wait this long at each end, as in Scenarios.three. */
  static final double SCORE_RADIUS = 1.5;

  static final double DWELL_SECONDS = 0.5;

  /** Sim robots: 6 m/s^2 at a 4.5 m/s top speed. Ghosts keep that ratio at any top speed. */
  static final double ACCEL_PER_SPEED = 6.0 / 4.5;

  /** Position and heading gains of the simulator's controller (HolonomicController.defaults). */
  static final double KP = 3.0;

  static final double K_THETA = 5.0;

  private final FieldConfig field;
  private final SimWorld world;
  private final DefenseAssist assist;
  private final PinTimer pinWatch;
  private final int ghostCount;

  /**
   * @param selfPose where our robot is right now (blue frame)
   */
  public GhostSession(GhostSettings ghosts, SafetyConfig safety, Pose selfPose) {
    this.field = ghosts.layout().field();
    double cap = safety.speedCap();

    // Our own body exists only so ghosts have something to collide with. Its top speed is the
    // speed cap, so "driving at the cap" counts as full pushing effort for the pin referee.
    RobotBody self =
        new RobotBody(
            SELF_ID, selfPose, cap, safety.accelLimit(), safety.omegaCap(), safety.alphaLimit());

    List<SimWorld.Opponent> opponents = new ArrayList<>();
    List<Vec2> pickups = field.opponentPickups();
    int n = Math.min(ghosts.count(), pickups.size());
    for (int id = 0; id < n; id++) {
      Vec2 pickup = pickups.get(id);
      RobotBody body =
          new RobotBody(
              id,
              new Pose(pickup, 0.0),
              ghosts.maxSpeed(),
              ghosts.maxSpeed() * ACCEL_PER_SPEED,
              2.0 * Math.PI,
              4.0 * Math.PI);
      CyclerBehavior behavior =
          ghosts.kind() == Scenarios.OpponentKind.EVADER
              ? CyclerBehavior.evader(pickup, field.defendedGoal(), SCORE_RADIUS, DWELL_SECONDS)
              : new CyclerBehavior(pickup, field.defendedGoal(), SCORE_RADIUS, DWELL_SECONDS);
      opponents.add(new SimWorld.Opponent(body, behavior));
    }
    this.ghostCount = n;
    this.world = new SimWorld(field, self, opponents);

    // Same modes and parameters as the simulator, except that "our top speed" is the speed cap.
    // (The other numbers repeat CutoffMode.defaults() and ZoneMode.defaults().)
    this.assist =
        new DefenseAssist(
            List.of(
                LineMirrorMode.defaults(),
                new CutoffMode(1.1, cap, 0.3),
                ShadowMode.defaults(),
                new ZoneMode(4.0, 1.5, 2.0, 1.0, cap)),
            new HolonomicController(
                positionGain(safety.accelLimit(), cap), K_THETA, cap, safety.omegaCap()),
            // alpha 1: the assist drives fully. Override threshold 0: any driver translation that
            // survived the deadband is an override (SafetyGate applies the same rule to turning).
            new DriverBlend(1.0, 0.0),
            new PerfectTracker(world),
            field);
    // The pin guard backs off earlier the longer a back-off takes at these caps
    // (RulesGuard.safetyMarginFor: 1.82 s at 1 m/s, 2 m/s^2 and 0.1 s latency).
    this.assist.setDriveLimits(cap, safety.accelLimit(), SafetyConfig.ASSUMED_COMMAND_LATENCY_SEC);
    this.pinWatch = new PinTimer(PinRules.placeholder(), field.robotHalfSize());
  }

  /**
   * Position gain that cannot overshoot under the acceleration limit.
   *
   * <pre>
   *   The controller asks for v = kP * error, capped at speedCap. That starts to fall when
   *   error = speedCap / kP. Stopping from speedCap at accelLimit takes speedCap^2 / (2 * accelLimit)
   *   meters, which must fit:   speedCap^2 / (2 a) <= speedCap / kP   =>   kP <= 2 a / speedCap
   *
   *   kP = min(KP, 2 * accelLimit / speedCap)
   * </pre>
   */
  public static double positionGain(double accelLimit, double speedCap) {
    return Math.min(KP, 2.0 * accelLimit / speedCap);
  }

  /**
   * Advance the ghosts by dt with our robot's measured state, then update the pin display.
   *
   * @param desiredVelocity the velocity our robot is trying to drive at (m/s): for the assist,
   *     the command BEFORE {@link VirtualContact} held it back, so a stalled push still counts as
   *     pushing
   */
  public void step(Pose selfPose, Vec2 selfVelocity, Vec2 desiredVelocity, double dt) {
    world.stepWithRealSelf(selfPose, selfVelocity, desiredVelocity, dt);
    Map<Integer, Vec2> positions = new HashMap<>();
    for (SimWorld.Opponent o : world.opponents()) {
      positions.put(o.body().id(), o.body().position());
    }
    // Counted like RulesGuard counts (any touching counts), so this timer shows what the pin
    // guard is about to act on. The guard's own timer is private to phantom.core.
    pinWatch.update(world.timeSec(), selfPose.position(), positions);
  }

  /** Ghosts in the form {@link VirtualContact} needs. */
  public List<VirtualContact.Ghost> contactGhosts() {
    List<VirtualContact.Ghost> out = new ArrayList<>();
    for (SimWorld.Opponent o : world.opponents()) {
      RobotBody b = o.body();
      out.add(new VirtualContact.Ghost(b.position(), b.desiredVelocity(), b.maxSpeed()));
    }
    return out;
  }

  /** Ghost poses in id order (blue frame). */
  public List<Pose> ghostPoses() {
    List<Pose> out = new ArrayList<>();
    for (SimWorld.Opponent o : world.opponents()) {
      out.add(o.body().pose());
    }
    return out;
  }

  /** Contact time in the current pin episode with ghost id (s); 0 if none. */
  public double pinSeconds(int id) {
    return pinWatch.contactSeconds(id);
  }

  /** Longest current pin episode over all ghosts (s). */
  public double maxPinSeconds() {
    double max = 0.0;
    for (int id = 0; id < ghostCount; id++) {
      max = Math.max(max, pinWatch.contactSeconds(id));
    }
    return max;
  }

  /** True while we are touching a ghost whose pin timer is at or past warningSec. */
  public boolean pinWarning(double warningSec) {
    for (int id = 0; id < ghostCount; id++) {
      if (pinWatch.touching(id) && pinWatch.contactSeconds(id) >= warningSec) {
        return true;
      }
    }
    return false;
  }

  public int ghostCount() {
    return ghostCount;
  }

  public FieldConfig field() {
    return field;
  }

  public SimWorld world() {
    return world;
  }

  public DefenseAssist assist() {
    return assist;
  }
}
