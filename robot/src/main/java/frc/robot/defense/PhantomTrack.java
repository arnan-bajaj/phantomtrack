package frc.robot.defense;

import java.util.List;
import java.util.Optional;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.manager.DefenseAssist;
import phantom.core.modes.DefenseTarget;

/**
 * PhantomTrack on a robot: one call per 20 ms loop, in the blue frame.
 *
 * <pre>
 *   pose --> PoseMonitor --> SafetyGate.decide --> who drives?
 *
 *   ASSIST:      DefenseAssist (phantom.core) --> VirtualContact --> SafetyGate.filter --> command
 *   NOT_TELEOP:  STOP
 *   otherwise:   the driver's command, untouched
 *
 *   then the ghosts take one step, reacting to the real pose (teleop only: they freeze otherwise)
 * </pre>
 *
 * No WPILib in here. {@code frc.robot.DefenseTeleop} feeds it the controller and the drivetrain
 * and sends the result back out.
 */
public final class PhantomTrack {
  /**
   * Everything the pipeline needs for one loop. Poses, velocities and commands are blue frame.
   *
   * @param nowSec monotonic clock (s)
   * @param mode what the Driver Station says we are doing
   * @param pose from the drivetrain's pose estimator
   * @param velocity measured field velocity (m/s)
   * @param poseTimestampSec when the pose was last updated, same clock as nowSec
   * @param driver the driver's own command, already deadbanded and scaled
   * @param stickDeflection largest raw stick deflection, 0..1 (translation magnitude or |turn|)
   * @param trigger right trigger, 0..1
   */
  public record Input(
      double nowSec,
      RobotMode mode,
      Pose pose,
      Vec2 velocity,
      double poseTimestampSec,
      ChassisCommand driver,
      double stickDeflection,
      double trigger) {}

  /** Longest step the ghosts ever take (s). A stalled loop must not teleport them. */
  public static final double MAX_DT = 0.1;

  private SafetyConfig safety;
  private GhostSettings ghosts;
  private SafetyConfig wantSafety;
  private GhostSettings wantGhosts;
  private boolean resetRequested = false;

  private PoseMonitor monitor;
  private SafetyGate gate;
  private GhostSession session = null; // built on the first loop with a usable pose

  private int modeIndex = GhostSession.MODE_LINE_MIRROR;
  private boolean hold = false;
  private int focus = -1; // -1 = auto

  private double lastNow = Double.NaN;
  private ChassisCommand lastOutput = ChassisCommand.STOP;
  private SafetyGate.Decision decision = SafetyGate.Decision.NOT_TELEOP;
  private PoseMonitor.Fault currentFault = PoseMonitor.Fault.NONE;
  private boolean pinWarning = false;

  public PhantomTrack(SafetyConfig safety, GhostSettings ghosts) {
    this.safety = safety;
    this.ghosts = ghosts;
    this.wantSafety = safety;
    this.wantGhosts = ghosts;
    this.monitor = PoseMonitor.of(safety);
    this.gate = new SafetyGate(safety, fenceFor(safety, ghosts));
  }

  private static Geofence fenceFor(SafetyConfig s, GhostSettings g) {
    return Geofence.inside(
        g.layout().field(),
        s.areaMinX(),
        s.areaMinY(),
        s.areaMaxX(),
        s.areaMaxY(),
        s.fenceMargin(),
        s.accelLimit());
  }

  /** Run one loop. Returns the field-relative command (blue frame) to send to the drivetrain. */
  public ChassisCommand update(Input in) {
    double dt =
        Double.isNaN(lastNow) ? 0.0 : Math.max(0.0, Math.min(MAX_DT, in.nowSec() - lastNow));
    lastNow = in.nowSec();

    boolean poseFinite =
        in.pose() != null
            && in.pose().position().isFinite()
            && Double.isFinite(in.pose().heading())
            && in.velocity() != null
            && in.velocity().isFinite();
    boolean triggerHeld = in.trigger() > safety.triggerThreshold();

    // New settings and ghost resets wait until the driver is not holding the assist trigger, so
    // the target can never jump under a robot that is being driven by the assist.
    if (poseFinite && (session == null || (!triggerHeld && settingsPending()))) {
      rebuild(in.pose());
    }

    currentFault =
        monitor.update(
            in.nowSec(), in.pose(), in.poseTimestampSec(), lastOutput.velocity().norm());
    decision =
        gate.decide(
            in.mode(),
            in.trigger(),
            in.stickDeflection(),
            monitor.healthy() && poseFinite,
            poseFinite ? in.pose().position() : null);

    ChassisCommand driver = CommandLimits.sanitize(in.driver());
    ChassisCommand out = decision == SafetyGate.Decision.NOT_TELEOP ? ChassisCommand.STOP : driver;
    pinWarning = false;
    if (session != null && poseFinite) {
      boolean engaged = decision == SafetyGate.Decision.ASSIST;
      // The assist runs every loop, engaged or not, so its pin timers and target stay current.
      // When engaged the sticks are inside the deadband by definition, so it gets STOP as "the
      // driver's command" and returns its own.
      ChassisCommand raw =
          session
              .assist()
              .update(
                  session.world().timeSec(),
                  in.pose(),
                  in.velocity(),
                  engaged ? ChassisCommand.STOP : driver,
                  engaged);
      Vec2 pressing = out.velocity();
      if (engaged) {
        ChassisCommand want = CommandLimits.sanitize(raw);
        Vec2 held =
            VirtualContact.limit(
                want.velocity(),
                in.pose().position(),
                session.contactGhosts(),
                session.field().robotHalfSize(),
                safety.speedCap());
        out =
            gate.filter(
                new ChassisCommand(held, want.omega()), lastOutput, in.pose().position(), dt);
        pressing = want.velocity();
      }
      if (in.mode() == RobotMode.TELEOP) {
        if (dt > 0.0) {
          session.step(in.pose(), in.velocity(), pressing, dt);
        }
        pinWarning = session.pinWarning(safety.pinWarningSec());
      }
    }
    lastOutput = out;
    return out;
  }

  private void rebuild(Pose pose) {
    boolean monitorChanged =
        wantSafety.maxPoseAgeSec() != safety.maxPoseAgeSec()
            || wantSafety.maxPoseJumpMeters() != safety.maxPoseJumpMeters()
            || wantSafety.maxHeadingJumpRad() != safety.maxHeadingJumpRad()
            || wantSafety.frozenPoseSec() != safety.frozenPoseSec()
            || wantSafety.poseRecoverySec() != safety.poseRecoverySec();
    safety = wantSafety;
    ghosts = wantGhosts;
    resetRequested = false;
    if (monitorChanged) {
      monitor = PoseMonitor.of(safety);
    }
    gate = new SafetyGate(safety, fenceFor(safety, ghosts));
    session = new GhostSession(ghosts, safety, pose);
    if (focus >= session.ghostCount()) {
      focus = -1;
    }
    session.assist().setModeIndex(modeIndex);
    session.assist().setHoldEnabled(hold);
    session.assist().setFocusOpponentId(focus);
  }

  // ---------------- Settings (dashboard) ----------------

  /** Ask for new settings. They take effect on the next loop with the trigger released. */
  public void requestSettings(SafetyConfig newSafety, GhostSettings newGhosts) {
    wantSafety = newSafety;
    wantGhosts = newGhosts;
  }

  /** Send the ghosts back to their pickup spots on the next loop with the trigger released. */
  public void requestGhostReset() {
    resetRequested = true;
  }

  /** True while a settings change or ghost reset is waiting for the trigger to be released. */
  public boolean settingsPending() {
    return resetRequested || !wantSafety.equals(safety) || !wantGhosts.equals(ghosts);
  }

  // ---------------- Driver controls ----------------

  /** Pick a mode by its position (see GhostSession.MODE_*). Out-of-range values are ignored. */
  public void selectMode(int index) {
    if (index < 0 || index > GhostSession.MODE_ZONE) {
      return;
    }
    modeIndex = index;
    if (session != null) {
      session.assist().setModeIndex(index);
    }
  }

  public void toggleHold() {
    hold = !hold;
    if (session != null) {
      session.assist().setHoldEnabled(hold);
    }
  }

  /**
   * Step the focus: auto -> #0 -> #1 -> ... -> auto (direction +1) or the other way (-1).
   *
   * <pre>
   *   with n ghosts, focus is in {-1, 0, ..., n-1}:   next = (focus + 1 + direction) mod (n + 1) - 1
   * </pre>
   */
  public void cycleFocus(int direction) {
    int n = session == null ? 0 : session.ghostCount();
    focus = Math.floorMod(focus + 1 + (direction >= 0 ? 1 : -1), n + 1) - 1;
    if (session != null) {
      session.assist().setFocusOpponentId(focus);
    }
  }

  // ---------------- State for telemetry ----------------

  public SafetyGate.Decision decision() {
    return decision;
  }

  /** What phantom.core's assist did on the last loop (OFF until the first usable pose). */
  public DefenseAssist.State coreState() {
    return session == null ? DefenseAssist.State.OFF : session.assist().lastState();
  }

  /** One line for the dashboard, e.g. "ASSIST: ENGAGED" or "POSE_FAULT: POSITION_JUMP". */
  public String status() {
    switch (decision) {
      case ASSIST:
        return "ASSIST: " + coreState();
      case POSE_FAULT:
        return "POSE_FAULT: " + monitor.lastFault();
      case OUTSIDE_AREA:
        return gate.fence() == null ? "OUTSIDE_AREA: test area is not on the field" : "OUTSIDE_AREA";
      default:
        return decision.toString();
    }
  }

  /** The fault seen on the last pose sample (NONE if it looked fine). */
  public PoseMonitor.Fault currentFault() {
    return currentFault;
  }

  public boolean poseHealthy() {
    return monitor.healthy();
  }

  /** Where the active mode wants us, even when the assist is not driving. */
  public Optional<DefenseTarget> target() {
    return session == null ? Optional.empty() : session.assist().lastTarget();
  }

  public List<Pose> ghostPoses() {
    return session == null ? List.of() : session.ghostPoses();
  }

  /** Pin timer per ghost, in id order (s). */
  public double[] pinSeconds() {
    int n = session == null ? 0 : session.ghostCount();
    double[] out = new double[n];
    for (int id = 0; id < n; id++) {
      out[id] = session.pinSeconds(id);
    }
    return out;
  }

  public double maxPinSeconds() {
    return session == null ? 0.0 : session.maxPinSeconds();
  }

  /** True while touching a ghost past the warning threshold in teleop: rumble the controller. */
  public boolean pinWarning() {
    return pinWarning;
  }

  /** Pins a referee would have called so far in this session (contact while pressing, 3 s). */
  public int pinViolations() {
    return session == null ? 0 : session.world().pinViolations();
  }

  /** The ghost the current target is reacting to, or -1. */
  public int guardedId() {
    return target().map(DefenseTarget::opponentId).orElse(-1);
  }

  /** Driver-selected focus: a ghost id, or -1 for auto. */
  public int focus() {
    return focus;
  }

  public int modeIndex() {
    return modeIndex;
  }

  public String modeName() {
    String name =
        session == null ? "(waiting for pose)" : session.assist().activeMode().name();
    return hold ? name + " + hold" : name;
  }

  public boolean hold() {
    return hold;
  }

  /** The active field layout; null until the first usable pose. */
  public FieldConfig field() {
    return session == null ? null : session.field();
  }

  /** The active geofence; null if the configured test area does not overlap the field. */
  public Geofence fence() {
    return gate.fence();
  }

  public SafetyConfig safety() {
    return safety;
  }

  public GhostSettings ghosts() {
    return ghosts;
  }

  public double ghostTimeSec() {
    return session == null ? 0.0 : session.world().timeSec();
  }

  public ChassisCommand lastOutput() {
    return lastOutput;
  }
}
