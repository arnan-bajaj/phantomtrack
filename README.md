# PhantomTrack

Driver-assist defense for FRC. The robot tracks opponents and helps the driver play different kinds
of defense (man-to-man, zone, lane blocking), switching between modes on the fly.

Right now this is **Phase 1: simulator + algorithms**. The sim knows every robot's exact pose, so we
can develop and test defense logic before touching vision or the real robot.

## Run it

You need Java 17 or newer. The JDK that comes with WPILib works.

```bash
./gradlew run                          # interactive sim window
./gradlew run --args="--headless"      # 60 s run: no defense vs. every mode, prints metrics, saves snapshots/*.png
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
| M | next defense mode |
| 1 / 2 / 3 / 4 | Shadow / Interpose / Zone / Line mirror |
| P | pause |
| R | reset |

Pressing a drive key while holding SPACE is a **driver override**: you get full control back. The
HUD shows the assist state (OFF / ENGAGED / DRIVER_OVERRIDE / NO_TARGET).

## Defense modes

| # | Mode | What it does | Uses focus (F)? |
| --- | --- | --- | --- |
| 1 | Shadow | Stays a fixed distance in front of the opponent, between them and the goal (man-to-man). | yes |
| 2 | Interpose | Stands halfway along the opponent's path to the goal, taking the lane early instead of retreating. | yes |
| 3 | Zone | Waits near the goal (green circle). Meets whoever enters the zone, using intercept math, then returns. | no, picks its own threat |
| 4 | Line mirror | Stays on a fixed line in front of the goal (green line) and slides to match the opponent. | yes |

The formula for each mode is in its class comment in `src/main/java/phantom/core/modes/`.

## How it's organized

```
src/main/java/phantom/
  core/            Robot-ready code. ZERO dependencies (no WPILib, no AWT), so it drops into robot code later.
    geometry/      Vec2, Pose, Angles, Intercept (earliest-catch-time math)
    field/         FieldConfig: field size, goal, pickups (placeholder until 2027 kickoff)
    tracking/      TrackedRobot, OpponentTracker  <- the ONLY way defense code sees opponents
    modes/         DefenseMode interface + Shadow, Interpose, Zone, LineMirror modes
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
- [x] Phase 4: Interpose, Zone (intercept math), Line mirror modes
- [ ] Phase 5: target selection with hysteresis, confidence fallback, pin timer (RulesGuard)
- [ ] Phase 6: batch runs with CSV metrics
- [ ] Phase 7: real robot (AprilTag pose) with simulated ghost opponents
- [ ] Phase 8: vision-based opponent tracking

## Current findings

Headless 60 s run, three Cycler opponents (they drive straight to the goal and push through):

| | No defense | Shadow | Interpose | Zone | Line mirror |
| --- | --- | --- | --- | --- | --- |
| Total opponent cycles | 27 | 27 | 27 | 25 | 22 |
| Goal-side when guarding | – | 90% | 100% | 91% | 46% |
| Mean tracking error | – | 0.91 m | 0.17 m | 0.81 m | 0.96 m |
| Contact time | 1.3 s | 26.6 s | 7.7 s | 13.0 s | 36.2 s |

What this says, honestly:

- **Interpose positions best** (always goal-side, 0.17 m from its target) but costs Cyclers nothing,
  because they shove through anyone in the way.
- **Line mirror cuts the most cycles (27 to 22), but only by making contact for 36 s of 60.** In a
  real match that is pinning-foul risk, and the sim's pushing model is too simple to trust it.
- These numbers will change a lot once (a) opponents dodge (Evader), and (b) the contact model lets
  a defender holding position resist being pushed. Don't pick a "best mode" from this table yet.
