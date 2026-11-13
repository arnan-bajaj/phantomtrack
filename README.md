# PhantomTrack

Driver-assist defense for FRC. The robot tracks opponents and helps the driver play different kinds
of defense (man-to-man, zone, lane blocking), switching between modes on the fly.

Right now this is **Phase 1: simulator + algorithms**. The sim knows every robot's exact pose, so we
can develop and test defense logic before touching vision or the real robot.

## Run it

You need Java 17 or newer. The JDK that comes with WPILib works.

```bash
./gradlew run                          # interactive sim window
./gradlew run --args="--headless"      # 60 s run: no defense vs. Shadow, prints metrics, saves snapshots/shadow-midrun.png
./gradlew test                         # unit tests
```

On Windows use `gradlew.bat` instead of `./gradlew`. The first run downloads Gradle and JUnit
(needs internet once).

If you see `JAVA_HOME is not set`, point it at WPILib's JDK, for example
`export JAVA_HOME=~/wpilib/2026/jdk` (macOS/Linux), or open the folder in WPILib VS Code and use its
terminal.

### Controls (sim window)

| Key | Action |
| --- | --- |
| W A S D | drive (field-relative, full speed) |
| Q / E | rotate |
| hold SPACE | engage the defense assist |
| F | cycle focus opponent: auto (nearest) → #0 → #1 → #2 → auto |
| M | next defense mode (only Shadow exists so far) |
| P | pause |
| R | reset |

Pressing a drive key while holding SPACE is a **driver override**: you get full control back. The
HUD shows the assist state (OFF / ENGAGED / DRIVER_OVERRIDE / NO_TARGET).

## How it's organized

```
src/main/java/phantom/
  core/            Robot-ready code. ZERO dependencies (no WPILib, no AWT), so it drops into robot code later.
    geometry/      Vec2, Pose, Angles
    field/         FieldConfig: field size, goal, pickups (placeholder until 2027 kickoff)
    tracking/      TrackedRobot, OpponentTracker  <- the ONLY way defense code sees opponents
    modes/         DefenseMode interface, ShadowMode
    control/       HolonomicController (target -> velocity), DriverBlend (driver override)
    manager/       DefenseAssist: tracker -> mode -> controller -> blend, once per 20 ms loop
  sim/             Simulator only. Never used on the robot.
    RobotBody      kinematic swerve with speed/accel limits
    CyclerBehavior opponent bot: pickup -> goal -> repeat
    SimWorld       ground truth + collisions
    PerfectTracker OpponentTracker that reports exact truth (noisy version comes in Phase 2)
    ui/            Swing window + renderer
```

The key idea: `DefenseAssist` gets opponents through `OpponentTracker`. Today that is
`PerfectTracker` (sim truth). Later it becomes a noisy sim tracker, then a second robot's odometry
over NetworkTables, then vision. Nothing in `core/modes` or `core/control` changes when we swap it.

## Roadmap

- [x] Phase 1: sim world, Cycler opponents, Shadow mode, controller, driver override, metrics
- [ ] Phase 2: noisy/laggy tracker + Kalman filter for opponent velocity
- [ ] Evader opponent (dodges the defender), so modes are tested against someone fighting back
- [ ] Phase 4: Interpose, Zone (intercept math), Line mirror modes
- [ ] Phase 5: target selection with hysteresis, confidence fallback, pin timer (RulesGuard)
- [ ] Phase 6: batch runs with CSV metrics
- [ ] Phase 7: real robot (AprilTag pose) with simulated ghost opponents
- [ ] Phase 8: vision-based opponent tracking

## Current findings

Headless 60 s run, three Cycler opponents:

| | No defense | Shadow |
| --- | --- | --- |
| Cycles per opponent | 9 / 9 / 9 | 9 / 9 / 9 |
| Goal-side of guarded opponent | – | 90% |
| Mean tracking error | – | 0.91 m |
| Contact time | – | 26.6 s |

Shadow holds the goal-side position as designed, but it **does not slow down a bot that bulldozes
straight to the goal**. The current sim lets the opponent shove us out of the way on contact.
Getting real cycle-time impact needs (a) an Evader opponent that has to path around us, (b) modes
that take space instead of retreating (Interpose), and (c) a contact model where a defender holding
position actually resists being pushed.
