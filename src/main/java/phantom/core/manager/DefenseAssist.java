package phantom.core.manager;

import java.util.List;
import java.util.Optional;
import phantom.core.control.ChassisCommand;
import phantom.core.control.DriverBlend;
import phantom.core.control.HolonomicController;
import phantom.core.control.RulesGuard;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseContext;
import phantom.core.modes.DefenseMode;
import phantom.core.modes.DefenseTarget;
import phantom.core.modes.HoldGroundMode;
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
 */
public final class DefenseAssist {
  /** What happened on the last update, for logging and the sim HUD. */
  public enum State {
    OFF,
    ENGAGED,
    DRIVER_OVERRIDE,
    NO_TARGET,
    /** RulesGuard is backing us off an opponent before the pin limit. */
    BACKING_OFF
  }

  private final List<DefenseMode> modes;
  private final HolonomicController controller;
  private final DriverBlend blend;
  private final OpponentTracker tracker;
  private final FieldConfig field;

  private RulesGuard guard; // null = disabled
  private boolean holdEnabled = false;
  private int modeIndex = 0;
  private int focusOpponentId = -1; // -1 = auto (nearest)
  private Optional<DefenseTarget> lastTarget = Optional.empty();
  private State lastState = State.OFF;

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
    this.guard = RulesGuard.defaults(field);
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
    List<TrackedRobot> opponents = tracker.getOpponents(nowSec);
    DefenseContext ctx =
        new DefenseContext(nowSec, self, selfVelocity, opponents, field, focusOpponentId);
    DefenseMode mode = holdEnabled ? HoldGroundMode.wrap(activeMode()) : activeMode();
    lastTarget = mode.compute(ctx);
    // The guard runs every loop (even when not engaged) so its pin timers stay accurate.
    if (guard != null) {
      lastTarget = guard.apply(nowSec, self, opponents, lastTarget);
    }

    if (!engaged) {
      lastState = State.OFF;
      return driver;
    }
    if (lastTarget.isEmpty()) {
      lastState = State.NO_TARGET;
      return driver;
    }
    if (blend.isOverriding(driver, controller.maxSpeed())) {
      lastState = State.DRIVER_OVERRIDE;
      return driver;
    }
    lastState = guard != null && guard.backingOff() ? State.BACKING_OFF : State.ENGAGED;
    ChassisCommand assist = controller.calculate(self, lastTarget.get());
    return blend.blend(driver, assist, true, controller.maxSpeed());
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

  /** -1 = auto (nearest opponent). */
  public void setFocusOpponentId(int id) {
    this.focusOpponentId = id;
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
      guard = RulesGuard.defaults(field);
    } else if (!enabled) {
      guard = null;
    }
  }

  public boolean rulesGuardEnabled() {
    return guard != null;
  }

  /** Number of times the guard forced a back-off (0 if disabled). */
  public int backoffsTriggered() {
    return guard == null ? 0 : guard.backoffsTriggered();
  }

  public Optional<DefenseTarget> lastTarget() {
    return lastTarget;
  }

  public State lastState() {
    return lastState;
  }
}
