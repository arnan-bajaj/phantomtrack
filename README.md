# PhantomTrack

Driver-assist defense for FRC. The robot tracks opponents and helps the driver play different kinds
of defense (man-to-man, zone, lane blocking), switching between modes on the fly.

Right now this is **Phase 1: simulator + algorithms**. The sim knows every robot's exact pose, so we
can develop and test defense logic before touching vision or the real robot.

## Run it

You need Java 17 or newer. The JDK that comes with WPILib works.

```bash
./gradlew run                          # interactive sim window
./gradlew run --args="--headless"      # 180 s: no defense vs. every mode, both opponent types; saves snapshots/*.png
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
| J / L | turn left / right |
| hold SPACE | engage the defense assist |
| F | cycle focus opponent: auto (nearest) → #0 → #1 → #2 → auto |
| O | switch opponent type (Cycler ↔ Evader) and restart |
| M | next defense mode |
| H | toggle **hold ground** (stop retreating on contact, push back) |
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

**Hold ground (H)** wraps any mode: once the opponent it's guarding gets within 0.3 m (bumper to
bumper) and we're goal-side, the robot stops retreating and pushes straight back.

**Pin guard (`RulesGuard`, always on)** times contact with each opponent. At 2.5 s (0.5 s before the
3 s placeholder limit) it steps the robot *sideways* out of the opponent's path until they are
1.83 m apart. Limits live in `PinRules` and must be updated from the 2027 manual.

## How it's organized

```
src/main/java/phantom/
  core/            Robot-ready code. ZERO dependencies (no WPILib, no AWT), so it drops into robot code later.
    geometry/      Vec2, Pose, Angles, Intercept (earliest-catch-time math)
    field/         FieldConfig: field size, goal, pickups (placeholder until 2027 kickoff)
    tracking/      TrackedRobot, OpponentTracker  <- the ONLY way defense code sees opponents
    modes/         DefenseMode interface + Shadow, Interpose, Zone, LineMirror modes
    control/       HolonomicController (target -> velocity), DriverBlend (driver override), RulesGuard (pin limit)
    rules/         PinRules (game-manual numbers), PinTimer (contact time per opponent)
    manager/       DefenseAssist: tracker -> mode -> controller -> blend, once per 20 ms loop
  sim/             Simulator only. Never used on the robot.
    RobotBody      kinematic swerve with speed/accel limits
    CyclerBehavior opponent bot: pickup -> goal -> repeat (Cycler pushes through, Evader dodges)
    Avoidance      Evader's steering around robots in its path
    Contact        pushing-match model: whoever drives harder into the other wins
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
- [x] Evader opponent (dodges the defender) and a pushing-match contact model
- [x] Phase 4: Interpose, Zone (intercept math), Line mirror modes
- [x] Hold-ground behavior + pin timer (RulesGuard) with sideways back-off
- [ ] Phase 5: target selection with hysteresis, confidence fallback
- [ ] Phase 6: batch runs with CSV metrics
- [ ] Phase 7: real robot (AprilTag pose) with simulated ghost opponents
- [ ] Phase 8: vision-based opponent tracking

## Current findings

Headless 180 s runs (`./gradlew run --args="--headless"`). Man-to-man modes guard opponent #0;
Zone picks its own threat. "No defense" parks our robot in a corner. **Slowed** = how much the
guarded opponent's average cycle time went up. **Pins** are counted like a referee would: only
contact time while we are driving into that opponent, one per episode that reaches 3 s.

| Setup | vs Cyclers (push through) | vs Evaders (dodge) | Pins (C / E) |
| --- | --- | --- | --- |
| Shadow | +6% | +9% | 0 / 0 |
| **Shadow + hold** | **+23%** | **+30%** | **0 / 0** |
| Interpose | −1% | −3% | 0 / 0 |
| Interpose + hold | +6% | +3% | 0 / 0 |
| Zone | +3% | +0% | 0 / 0 |
| Zone + hold | +4% | +1% | 0 / 0 |
| Line mirror | +35% | +24% | 0 / 0 |
| Line mirror + hold | +38% | +28% | 0 / 0 |
| Shadow + hold, **no pin guard** | +28% | +126% | 9 / 3 |
| Line mirror + hold, **no pin guard** | +50% | +27% | 17 / 0 |

What this says:

- **Holding ground is what makes defense work.** Shadow goes from +6–9% to +23–30% just by not
  retreating on contact, and stays legal.
- **The pin guard costs a little effectiveness and removes every pin.** Without it, the numbers
  look better (Shadow + hold jams an Evader for 166 of 180 s) but every one of those runs would
  draw fouls.
- **Line mirror and Shadow + hold are the two strongest legal setups.** Line mirror is better
  against bulldozers, Shadow + hold against robots that dodge.
- **Interpose and Zone barely matter** in this scenario. Both stand far from the opponent, so they
  rarely get the contact that hold needs.
- Limits: one defender, three opponents with fixed routes, no game pieces, idealized pushing. Treat
  the ranking as a direction, not a guarantee.
