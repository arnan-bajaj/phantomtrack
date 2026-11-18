package phantom.core.manager;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import phantom.core.control.ChassisCommand;
import phantom.core.control.DriverBlend;
import phantom.core.control.HolonomicController;
import phantom.core.control.RulesGuard;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseContext;
import phantom.core.modes.DefenseMode;
import phantom.core.modes.DefenseTarget;
import phantom.core.modes.HoldGroundMode;
import phantom.core.rules.ContactDetector;
import phantom.core.rules.ContactEstimate;
import phantom.core.rules.PinRules;
import phantom.core.rules.DriveTelemetry;
import phantom.core.tracking.OpponentTracker;
import phantom.core.tracking.TrackedRobot;

/**
 * The whole driver-assist pipeline in one place, called once per loop (20 ms):
 *
 * <pre>
 *   tracker -> context -> active mode (+ hold ground) -> rules guard -> controller
 *           -> blend with driver -> command
 * </pre>
 *
 * The same object will run inside the robot's teleop loop later; only the tracker changes.
 *
 * <p>Tracks with a NaN or infinite position, velocity or time are dropped before anything else
 * sees them, whatever the tracker: a NaN must never reach a command.
 */
public final class DefenseAssist {
  /** What happened on the last update, for logging and the sim HUD. */
  public enum State {
    OFF,
    ENGAGED,
    DRIVER_OVERRIDE,
    /** Engaged but nobody to defend: returning to the home spot and scanning. */
    SEARCHING,
    /** RulesGuard is backing us off an opponent before the pin limit. */
    BACKING_OFF
  }

  private final List<DefenseMode> modes;
  private final HolonomicController controller;
  private final DriverBlend blend;
  private final OpponentTracker tracker;
  private final FieldConfig field;

  private RulesGuard guard; // null = disabled
  private ContactDetector.Config detectorConfig = ContactDetector.Config.placeholder();
  private double pinSafetyMargin;
  private double backoffSpeed = RulesGuard.SIM_SPEED;
  private double backoffAccel = RulesGuard.SIM_ACCEL;
  private boolean holdEnabled = false;
  private int modeIndex = 0;
  private int focusOpponentId = -1; // -1 = auto
  private final TargetSelector selector = TargetSelector.defaults();
  private boolean useThreatSelector = true; // auto focus: threat-based with hysteresis (else nearest)
  private Optional<DefenseTarget> lastTarget = Optional.empty();
  private List<TrackedRobot> lastOpponents = List.of();
  private ChassisCommand lastCommand = ChassisCommand.STOP;
  private State lastState = State.OFF;
  private DriveTelemetry driveTelemetry = null; // set before each update; null = not provided

  public DefenseAssist(
      List<DefenseMode> modes,
      HolonomicController controller,
      DriverBlend blend,
      OpponentTracker tracker,
      FieldConfig field) {
    if (modes.isEmpty()) {
      throw new IllegalArgumentException("need at least one mode");
    }
    this.modes = List.copyOf(modes);
    this.controller = controller;
    this.blend = blend;
    this.tracker = tracker;
    this.field = field;
    this.pinSafetyMargin = RulesGuard.simMargin(PinRules.placeholder(), field.robotHalfSize());
    this.guard = newGuard();
  }

  /**
   * @param nowSec current time (s)
   * @param self our pose
   * @param selfVelocity our field velocity (m/s)
   * @param driver the driver's own command from the sticks
   * @param engaged true while the driver holds the assist button
   */
  public ChassisCommand update(
      double nowSec, Pose self, Vec2 selfVelocity, ChassisCommand driver, boolean engaged) {
    List<TrackedRobot> opponents = finiteOnly(tracker.getOpponents(nowSec));
    lastOpponents = opponents;
    int focus = focusOpponentId;
    if (focus < 0 && useThreatSelector) {
      focus = selector.select(nowSec, opponents, field.defendedGoal());
    }
    DefenseContext ctx = new DefenseContext(nowSec, self, selfVelocity, opponents, field, focus);
    DefenseMode mode = holdEnabled ? HoldGroundMode.wrap(activeMode()) : activeMode();
    lastTarget = mode.compute(ctx);
    // The guard runs every loop (even when not engaged) so its pin timers stay accurate.
    if (guard != null) {
      lastTarget =
          driveTelemetry != null
              ? guard.apply(nowSec, self, opponents, lastTarget, driveTelemetry)
              : guard.apply(nowSec, self, opponents, lastTarget, lastCommand.velocity(), selfVelocity);
    }
    driveTelemetry = null; // used once: a stale sample must never be mistaken for a fresh one
    lastCommand = decide(nowSec, self, driver, engaged);
    return lastCommand;
  }

  /** The tracks whose position, velocity and time are all finite numbers. */
  static List<TrackedRobot> finiteOnly(List<TrackedRobot> tracks) {
    List<TrackedRobot> out = new ArrayList<>(tracks.size());
    for (TrackedRobot r : tracks) {
      if (r != null
          && r.position().isFinite()
          && r.velocity().isFinite()
          && Double.isFinite(r.timestampSec())) {
        out.add(r);
      }
    }
    return out;
  }

  private ChassisCommand decide(double nowSec, Pose self, ChassisCommand driver, boolean engaged) {
    if (!engaged) {
      lastState = State.OFF;
      return driver;
    }
    if (blend.isOverriding(driver, controller.maxSpeed(), controller.maxOmega())) {
      lastState = State.DRIVER_OVERRIDE;
      return driver;
    }
    if (lastTarget.isEmpty()) {
      lastState = State.SEARCHING;
      ChassisCommand search = controller.calculate(self, routeAroundObstacles(self, searchTarget(nowSec)));
      return blend.blend(driver, search, true, controller.maxSpeed(), controller.maxOmega());
    }
    boolean backing = guard != null && (guard.backingOff() || guard.blindBackingOff(nowSec));
    lastState = backing ? State.BACKING_OFF : State.ENGAGED;
    ChassisCommand assist = controller.calculate(self, routeAroundObstacles(self, lastTarget.get()));
    return blend.blend(driver, assist, true, controller.maxSpeed(), controller.maxOmega());
  }

  /**
   * Where to wait and look when there is nothing to defend:
   *
   * <pre>
   *   d       = unit(fieldCenter - goal)
   *   home    = goal + HOME_DISTANCE * d
   *   heading = wrap(angle(d) + SCAN_AMPLITUDE * sin(SCAN_RATE * t))   (sweep the cameras)
   * </pre>
   */
  public DefenseTarget searchTarget(double nowSec) {
    Vec2 goal = field.defendedGoal();
    Vec2 d = new Vec2(field.length() / 2.0, field.width() / 2.0).minus(goal).normalized();
    if (d.equals(Vec2.ZERO)) {
      d = new Vec2(-1.0, 0.0);
    }
    Vec2 home = field.clampInside(goal.plus(d.times(HOME_DISTANCE)));
    double heading = Angles.wrap(d.angle() + SCAN_AMPLITUDE * Math.sin(SCAN_RATE * nowSec));
    return new DefenseTarget(home, heading, Vec2.ZERO, -1);
  }

  public static final double HOME_DISTANCE = 2.5; // m
  public static final double SCAN_AMPLITUDE = 0.6; // rad (about 34 deg each way)
  public static final double SCAN_RATE = 0.8; // rad/s

  /**
   * If a wall is between us and the target, go via the best gate instead.
   *
   * <pre>
   *   if path(self -> target) is blocked:
   *     gate* = argmin over gates of |self - g| + |g - target|
   *     drive to gate* (no feedforward), keep the target's heading
   * </pre>
   */
  public DefenseTarget routeAroundObstacles(Pose self, DefenseTarget target) {
    if (field.gates().isEmpty() || !field.pathBlocked(self.position(), target.position())) {
      return target;
    }
    Vec2 best = null;
    double bestCost = Double.POSITIVE_INFINITY;
    for (Vec2 g : field.gates()) {
      double c = self.position().distanceTo(g) + g.distanceTo(target.position());
      if (c < bestCost) {
        bestCost = c;
        best = g;
      }
    }
    return new DefenseTarget(best, target.heading(), Vec2.ZERO, target.opponentId());
  }

  public DefenseMode activeMode() {
    return modes.get(modeIndex);
  }

  public void nextMode() {
    modeIndex = (modeIndex + 1) % modes.size();
  }

  /** Select a mode by its position in the list (0-based). Out-of-range indexes are ignored. */
  public void setModeIndex(int index) {
    if (index >= 0 && index < modes.size()) {
      modeIndex = index;
    }
  }

  public int modeIndex() {
    return modeIndex;
  }

  public List<DefenseMode> modes() {
    return modes;
  }

  public int focusOpponentId() {
    return focusOpponentId;
  }

  /** -1 = auto (threat-based selection, or nearest if the selector is off). */
  public void setFocusOpponentId(int id) {
    this.focusOpponentId = id;
  }

  /** Auto focus picks by threat with hysteresis (true, default) or just the nearest (false). */
  public void setThreatSelectorEnabled(boolean enabled) {
    this.useThreatSelector = enabled;
  }

  public boolean threatSelectorEnabled() {
    return useThreatSelector;
  }

  /** How many times auto focus changed targets. */
  public int autoTargetSwitches() {
    return selector.switches();
  }

  public boolean holdEnabled() {
    return holdEnabled;
  }

  public void setHoldEnabled(boolean enabled) {
    this.holdEnabled = enabled;
  }

  /** Pin-limit guard; true by default. Turning it off is for experiments only. */
  public void setRulesGuardEnabled(boolean enabled) {
    if (enabled && guard == null) {
      guard = newGuard();
    } else if (!enabled) {
      guard = null;
    }
  }

  /**
   * Contact detector thresholds (calibrated per robot and camera). Resets the pin guard's timers,
   * so set it before the match starts.
   */
  public void setContactDetectorConfig(ContactDetector.Config config) {
    this.detectorConfig = config;
    if (guard != null) {
      guard = newGuard();
    }
  }

  private RulesGuard newGuard() {
    RulesGuard g = new RulesGuard(PinRules.placeholder(), field, pinSafetyMargin, detectorConfig);
    g.setIdsAreStable(tracker.idsAreStable());
    g.setDriveLimits(backoffSpeed, backoffAccel);
    return g;
  }

  /**
   * Tell the pin guard how fast this robot can back off: its speed cap (m/s), acceleration limit
   * (m/s^2) and command latency (s, beyond the 20 ms loop). The guard then starts backing off
   * {@link RulesGuard#safetyMarginFor} before the limit. Without this call it uses the simulator
   * robot's numbers ({@link RulesGuard#simMargin}). Resets the pin guard's timers, so set it before
   * the match starts.
   */
  public void setDriveLimits(double maxSpeed, double maxAccel, double latencySec) {
    this.pinSafetyMargin = RulesGuard.safetyMarginFor(
        PinRules.placeholder(), field.robotHalfSize(), maxSpeed, maxAccel, latencySec);
    if (maxSpeed > 0.0 && maxAccel > 0.0) {
      this.backoffSpeed = maxSpeed;
      this.backoffAccel = maxAccel;
    }
    if (guard != null) {
      guard = newGuard();
    }
  }

  /** True if the pin guard is backing off with no safe exit (yielding) on the last update. */
  public boolean guardYielding() {
    return guard != null && guard.yielding();
  }

  /** Pin time the guard has counted in this opponent's open episode (s); 0 if disabled. */
  public double pinSeconds(int opponentId) {
    return guard == null ? 0.0 : guard.contactSeconds(opponentId);
  }

  /** How long before the pin limit the guard starts backing off (s). */
  public double pinSafetyMargin() {
    return pinSafetyMargin;
  }

  public boolean rulesGuardEnabled() {
    return guard != null;
  }

  /**
   * Drive telemetry for contact sensing (commanded and measured velocity, acceleration, drive
   * current). Call before every {@link #update}; without it the guard falls back to velocities
   * only.
   */
  public void setDriveTelemetry(DriveTelemetry telemetry) {
    this.driveTelemetry = telemetry;
  }

  /** What the contact detector believed on the last update (nothing if the guard is off). */
  public ContactEstimate lastContact() {
    return guard == null ? ContactEstimate.none() : guard.lastContact();
  }

  /** True if the pin guard was counting contact time on the last update (false if disabled). */
  public boolean guardSensesContact() {
    return guard != null && guard.timingContact();
  }

  /** Number of times the guard forced a back-off (0 if disabled). */
  public int backoffsTriggered() {
    return guard == null ? 0 : guard.backoffsTriggered();
  }

  /** The opponent tracks used on the last update (what the robot "believes"). */
  public List<TrackedRobot> lastOpponents() {
    return lastOpponents;
  }

  public Optional<DefenseTarget> lastTarget() {
    return lastTarget;
  }

  public State lastState() {
    return lastState;
  }
}
