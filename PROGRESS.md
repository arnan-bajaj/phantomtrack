# Phase 7 progress (branch `phase7-robot`)

Goal: PhantomTrack on a real swerve robot, using its real pose, defending against simulated ghost
opponents shown in AdvantageScope. Safe enough for a first test on carpet.

## Done

- **Core purity test** (`CorePurityTest`): nothing under `phantom/core` may refer to `java.awt`,
  `javax.swing`, `edu.wpi`, `com.ctre` or `phantom.sim`.
- **Ghosts against a real pose**: `SimWorld.stepWithRealSelf` / `RobotBody.setMeasured`. The
  simulator's step() is unchanged (headless tables byte-identical to `main`).
- **`robot/`**: WPILib 2026.2.1 command-based project. Compiles `phantom/core/**` and the
  AWT-free ghost classes listed in `robot/shared-sources.txt` from `../src/main/java`
  (`RobotSharedSourcesTest` guards the list).
- **WPILib-free robot logic** (`frc.robot.defense`, `DefensePackagePurityTest`): `SafetyGate`,
  `PoseMonitor`, `CommandLimits`, `Geofence`, `FieldMirror`, `VirtualContact` (two-sided, the
  sim's `Contact` model), `GhostSession`, `DriverInput`, `PhantomTrack` pipeline. 76 tests.
- **WPILib layer**: `DefenseDrive`, `SimDefenseDrive`, `AllianceFrame`, `DefenseTeleop`,
  `RobotContainer` (Xbox bindings), `DefenseDashboard`, `DefenseTelemetry`.
- **AdvantageScope layout** `robot/advantagescope-layout.json` (26.0.0 format).
- **Desktop sim run** (headless WPILib sim + scripted DS/Xbox, `robot/tools/SimDriveCheck.java`).
  All phases behaved as expected; see the PR for exactly what was checked.
- **Docs**: `robot/README.md` (adapters for CTRE Tuner X and YAGSL, 2027 notes),
  `docs/phase7-test-plan.md`.

## Next

1. Open the layout in AdvantageScope once (only checked against its source code so far).
2. Drive the GUI sim by hand with a real Xbox controller (the scripted run covered the same steps).
3. Wire the team's real drivetrain adapter, then follow `docs/phase7-test-plan.md`.

## Open questions

- **Pin guard margin at test speeds (core change).** `RulesGuard`'s 0.5 s margin assumes a
  4.5 m/s, 6 m/s^2 robot. At the first-test caps the any-touch timer reached 3.2–3.7 s during
  back-offs in the desktop sim (the referee count stayed 0). Proposal: let `DefenseAssist` take a
  `RulesGuard` (or a safety margin), so the robot can set it from its real speed and acceleration.
- **Back-off into the geofence.** The guard's sideways back-off can point at the fence; the robot
  then waits at the fence until the back-off times out. Same open problem as back-offs into walls.
- **Pose timestamps.** YAGSL does not expose when odometry last ran, so only the frozen-pose check
  catches a dead estimator there.
- **Field symmetry for 2027** (`Constants.MIRROR`: rotated or mirrored) comes from the manual.
- The geofence's braking curve, sampled every 20 ms, can cross the fence line by up to 0.4 mm
  (derived in `GeofenceTest`). Harmless; noted so nobody is surprised.
