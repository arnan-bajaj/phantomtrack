# Phase 7 progress (branch `phase7-robot`)

Goal: PhantomTrack on a real swerve robot, using its real pose, defending against simulated ghost
opponents shown in AdvantageScope. Safe enough for a first test on carpet.

## Done

- **Core purity test** (`src/test/java/phantom/core/CorePurityTest.java`): fails if anything under
  `phantom/core` refers to `java.awt`, `javax.swing`, `edu.wpi`, `com.ctre` or `phantom.sim`
  (imports and inline qualified names; comments ignored).
- **Ghosts against a real pose** (`SimWorld.stepWithRealSelf`, `RobotBody.setMeasured`): the sim's
  own opponents, pushing model and pin referee run unchanged, but our robot's state is measured
  and a ghost can never move it. 9 hand-computed tests in `RealSelfTest`. Headless metrics are
  byte-identical to `main`.

- **`robot/` project** (GradleRIO 2026.2.1, matches `~/wpilib/2026`): compiles `phantom/core/**`
  plus the AWT-free ghost classes listed in `robot/shared-sources.txt` straight from
  `../src/main/java`. `RobotSharedSourcesTest` (main project) keeps that list robot-safe.
- **WPILib-free robot logic** in `robot/.../frc/robot/defense/` (compiles; partly tested):
  `Geofence`, `CommandLimits` (speed cap, accel limit, NaN -> STOP), `PoseMonitor`
  (stale / jump / heading jump / frozen / NaN), `SafetyGate` (teleop-only, deadman, latched pose
  fault, override, test area), `FieldMirror` (red alliance), `VirtualContact` (ghosts push back via
  the sim's `Contact` model), `GhostSession`, `PhantomTrack` (the per-loop pipeline).
- Tests so far: `GeofenceTest`, `CommandLimitsTest` (18 robot tests).

## Next

1. Tests: PoseMonitor, SafetyGate, FieldMirror (hand-computed red values), VirtualContact,
   PhantomTrack pipeline (deadman, override, autonomous, pose jump), no-WPILib-in-`defense` rule.
2. WPILib layer: `DefenseDrive`, kinematic sim drivetrain, alliance adapter, Xbox bindings,
   rumble, NetworkTables telemetry, dashboard choosers.
3. Desktop sim run, AdvantageScope layout, `robot/README.md`, `docs/phase7-test-plan.md`, PR.

## Open questions

- The geofence brakes along sqrt(2 a d) in 20 ms steps, so the last step can cross the fence
  line by up to 0.4 mm before stopping (derived in `GeofenceTest`). Harmless inside a 0.75 m
  margin; noted so nobody is surprised.
