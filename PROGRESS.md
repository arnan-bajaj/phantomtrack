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

## Next

1. `robot/` GradleRIO 2026.2.1 project that compiles `phantom/core/**` (plus the AWT-free ghost
   classes from `phantom/sim`) straight from `../src/main/java`.
2. WPILib-free safety layer + tests: deadman, override, speed cap, accel limit, geofence, pose
   health, teleop-only.
3. Alliance mirroring, `DefenseDrive` boundary, kinematic sim drivetrain, Xbox bindings, telemetry.
4. Desktop sim run, AdvantageScope layout, `robot/README.md`, `docs/phase7-test-plan.md`, PR.

## Open questions

- none yet
