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
| Q / E | rotate |
| hold SPACE | engage the defense assist |
| F | cycle focus opponent: auto (nearest) → #0 → #1 → #2 → auto |
| O | switch opponent type (Cycler ↔ Evader) and restart |
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
- [ ] Phase 5: target selection with hysteresis, confidence fallback, pin timer (RulesGuard)
- [ ] Phase 6: batch runs with CSV metrics
- [ ] Phase 7: real robot (AprilTag pose) with simulated ghost opponents
- [ ] Phase 8: vision-based opponent tracking

## Current findings

Headless 180 s runs. Man-to-man modes guard opponent #0; Zone picks its own threat. "No defense"
parks our robot in a corner. The key number is how much the guarded opponent's average cycle time
goes up.

**Cycler opponents** (drive straight, push through):

| | No defense | Shadow | Interpose | Zone | Line mirror |
| --- | --- | --- | --- | --- | --- |
| Guarded opp avg cycle | 7.15 s | 7.29 s | 7.09 s | 7.36 s | 12.00 s |
| Slowed by | – | +2% | −1% | +3% | **+68%** |
| Total cycles (all 3 opps) | 76 | 75 | 77 | 74 | 66 |
| Contact time | 0.0 s | 9.1 s | 4.4 s | 35.3 s | 81.7 s |

**Evader opponents** (steer around robots in their way):

| | No defense | Shadow | Interpose | Zone | Line mirror |
| --- | --- | --- | --- | --- | --- |
| Guarded opp avg cycle | 7.17 s | 7.60 s | 6.99 s | 7.18 s | 8.91 s |
| Slowed by | – | +6% | −3% | +0% | **+24%** |
| Total cycles (all 3 opps) | 79 | 75 | 79 | 75 | 71 |
| Contact time | 0.0 s | 5.6 s | 2.9 s | 4.6 s | 32.7 s |

What this says:

- **Modes that keep a standoff distance (Shadow, Interpose, Zone) don't slow anyone.** When the
  opponent drives at them they back away to keep their distance, so they escort the opponent to
  the goal. Good positioning, zero cost to the opponent. Differences of a few percent are noise.
- **Line mirror is the only mode that costs cycles**, because it holds its line and pushes back.
  That also means the most contact (82 s of 180 against Cyclers), which is pinning-foul risk.
- **Takeaway: defense that works needs to hold ground on contact,** and that has to be limited by
  a pin timer. Next step: a "hold" behavior for the standoff modes plus a RulesGuard that backs off
  before the pin limit.
