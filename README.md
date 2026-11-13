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
./gradlew run --args="--batch"         # every setup x opponents x sensor x 3 seeds -> results/batch.csv
./gradlew run --args="--batch 180 10"  # same, 180 s per run, 10 seeds
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
| F | cycle focus opponent: auto → #0 → #1 → #2 → auto |
| O | switch opponent type (Cycler ↔ Evader) and restart |
| N | switch sensor: Perfect → Realistic → Bad camera, and restart |
| M | next defense mode |
| H | toggle **hold ground** (stop retreating on contact, push back) |
| 1 / 2 / 3 / 4 | Shadow / Interpose / Zone / Line mirror |
| P | pause |
| R | reset |

Pressing a drive key while holding SPACE is a **driver override**: you get full control back. The
HUD shows the assist state (OFF / ENGAGED / DRIVER_OVERRIDE / SEARCHING / BACKING_OFF). With a
noisy sensor, dashed outlines show where the robot *thinks* opponents are.

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

**Auto focus** (focus = auto): a `TargetSelector` picks the opponent most likely to score soon,
stays on it until it turns away from the goal, then picks up the next one. It only jumps early if
someone else will reach the goal at least 2 s sooner, for 0.5 s straight.

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
                   Measurement, MeasurementSource, AxisKalman, KalmanOpponentTracker (noisy sightings -> tracks)
    modes/         DefenseMode interface + Shadow, Interpose, Zone, LineMirror modes
    control/       HolonomicController (target -> velocity), DriverBlend (driver override), RulesGuard (pin limit)
    rules/         PinRules (game-manual numbers), PinTimer (contact time per opponent)
    manager/       DefenseAssist: tracker -> mode -> controller -> blend, once per 20 ms loop
                   TargetSelector: who to guard when focus is auto
  sim/             Simulator only. Never used on the robot.
    RobotBody      kinematic swerve with speed/accel limits
    CyclerBehavior opponent bot: pickup -> goal -> repeat (Cycler pushes through, Evader dodges)
    Avoidance      Evader's steering around robots in its path
    Contact        pushing-match model: whoever drives harder into the other wins
    SimWorld       ground truth + collisions
    PerfectTracker OpponentTracker that reports exact truth
    NoisySensor    simulated camera: FOV, range, noise, latency, dropouts (feeds KalmanOpponentTracker)
    BatchRunner    every combination x seeds -> CSV for graphing
    ui/            Swing window + renderer
```

The key idea: `DefenseAssist` gets opponents through `OpponentTracker`. Today that is
`PerfectTracker` (sim truth). Later it becomes a noisy sim tracker, then a second robot's odometry
over NetworkTables, then vision. Nothing in `core/modes` or `core/control` changes when we swap it.

## Roadmap

- [x] Phase 1: sim world, Cycler opponents, Shadow mode, controller, driver override, metrics
- [x] Phase 2: noisy/laggy camera + Kalman filter, search behavior, stall-based contact detection
- [x] Evader opponent (dodges the defender) and a pushing-match contact model
- [x] Phase 4: Interpose, Zone (intercept math), Line mirror modes
- [x] Hold-ground behavior + pin timer (RulesGuard) with sideways back-off
- [x] Phase 5: target selection (stay on scorer until it turns away), search when nothing is visible
- [x] Phase 6: batch runs with CSV metrics (`--batch`)
- [ ] Phase 7: real robot (AprilTag pose) with simulated ghost opponents
- [ ] Phase 8: vision-based opponent tracking

## Current findings

Headless 180 s runs (`./gradlew run --args="--headless"`). Man-to-man modes guard opponent #0;
Zone picks its own threat. "No defense" parks our robot in a corner. **Slowed** = how much the
guarded opponent's average cycle time went up. **Pins** are counted like a referee would: only
contact time while we are driving into that opponent, one per episode that reaches 3 s.

### With perfect tracking

| Setup | vs Cyclers (push through) | vs Evaders (dodge) | Pins (C / E) |
| --- | --- | --- | --- |
| Shadow | +2% | +7% | 0 / 0 |
| **Shadow + hold** | **+24%** | **+23%** | **0 / 0** |
| Interpose + hold | +5% | +6% | 0 / 0 |
| Zone + hold | +4% | +1% | 0 / 0 |
| Line mirror | +35% | +26% | 0 / 0 |
| **Line mirror + hold** | **+36%** | **+28%** | **0 / 0** |
| Shadow + hold, no pin guard | +28% | +126% | 9 / 3 |
| Line mirror + hold, no pin guard | +50% | +27% | 17 / 0 |

### With a simulated camera (averaged over 5 seeds)

"Realistic" = two cameras (140°), 0.10 m noise, 80 ms latency, 15% dropped frames, 8 m range.
"Bad" = one 70° camera, 0.25 m noise, 150 ms, 35% dropped, 6 m. These are guesses until the real
pipeline is measured.

| Opponents | Setup | Perfect | Realistic | Bad |
| --- | --- | --- | --- | --- |
| Cyclers | Shadow + hold | +24% | +14% | +10% |
| Cyclers | Line mirror + hold | +36% | +29% | +12% |
| Evaders | Shadow + hold | +23% | +12% | +10% |
| Evaders | Line mirror + hold | +28% | +15% | +9% |

Pins with a camera: 0 to 0.8 per 180 s. Position estimate error: about 0.4 m (Realistic), 0.9 m (Bad).
Time each opponent is tracked: 50–73% (Realistic), 31–41% (Bad).

What this says:

- **Holding ground is what makes defense work**, and the pin guard keeps it legal with perfect
  tracking.
- **A realistic camera costs about half the effect.** The robot only sees each opponent half the
  time. Line mirror holds up best because it faces down-field and only needs one coordinate.
- **Camera coverage matters more than camera accuracy.** Going from Realistic to Bad roughly
  doubles the error, but the bigger loss comes from seeing opponents less often. Wider field of
  view (more cameras) is the first thing to buy.
- **The pin guard can't be camera-only.** With noisy tracking it misses some contact and about one
  pin slips through every 4–5 minutes of defending. The robot's own sense of being blocked (pushing
  but not moving) already catches most of it. On the real robot, add contact sensing (drive motor
  current spikes) before trusting it in a match.
### Who to guard when nobody is chosen (effect on all three opponents' total cycles)

| Setup | vs Cyclers | vs Evaders |
| --- | --- | --- |
| Shadow + hold, locked on one scorer | **+9%** | +10% |
| Shadow + hold, auto = nearest | +4% | +10% |
| Shadow + hold, auto = threat | +7% | +5% |
| Line mirror + hold, locked on one scorer | +6% | +11% |
| Line mirror + hold, auto = nearest | +10% | +13% |
| Line mirror + hold, auto = threat | **+15%** | +13% |

- **Man-to-man (Shadow): pick one scorer and stay on them.** Switching costs more than it gains.
- **Positional defense (Line mirror): let it take whoever comes through** (auto = threat).
- Totals move by about ±1 cycle (≈ ±1.5%) between runs, so treat small gaps as ties.
- Oddity: plain Shadow does better with a noisy camera (+20%) than with perfect tracking (+2%).
  Tracking lag makes it drift into the opponent's path, which accidentally acts like hold.

- Limits: one defender, three opponents with fixed routes, no game pieces, idealized pushing,
  opponent identity assumed known. Treat rankings as direction, not guarantees.
