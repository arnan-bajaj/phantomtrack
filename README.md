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
./gradlew run --args="--anonymous"     # anonymous camera vs known ids, partners on (see docs/findings/)
./gradlew run --args="--tracker-sweep" # how the multi-target tracker's settings were chosen
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
| F | cycle focus opponent: auto → #0 → #1 → #2 → auto (anonymous tracking: a simulated driver keeps the assist on that robot's track) |
| O | switch opponent type (Cycler ↔ Evader) and restart |
| N | switch sensor: Perfect → Realistic → Bad camera, and restart |
| T | switch tracking: known ids → known ids + occlusion → anonymous (multi-target tracker), and restart |
| B | two alliance partners on / off, and restart |
| G | switch field: Open → Choke (one gap) → Two lanes, and restart |
| M | next defense mode |
| H | toggle **hold ground** (stop retreating on contact, push back) |
| 1 / 2 / 3 / 4 / 5 | Shadow / Interpose / Zone / Line mirror / Cut-off |
| P | pause |
| R | reset |

The window starts in **Shadow + hold, locked on opponent #0**, the strongest man-to-man setup.

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
| 5 | Cut-off | Predicts the opponent's route (destination + which gap) and waits at the first point on it we can reach before they do. | yes |

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
    field/         FieldConfig: field size, goal, pickups, obstacles + gates (placeholder until 2027 kickoff)
                   Obstacle: wall rectangle (collision, push-out, path-blocked checks)
    tracking/      TrackedRobot, OpponentTracker  <- the ONLY way defense code sees opponents
                   Measurement, MeasurementSource, AxisKalman, KalmanOpponentTracker (noisy sightings -> tracks)
                   Detection, DetectionFrame, MultiTargetTracker (anonymous detections -> tracks), with
                   Hungarian (assignment), Gating (Mahalanobis), ColorBelief (alliance color), MOfN
    modes/         DefenseMode interface + Shadow, Interpose, Zone, LineMirror, Cutoff modes
    prediction/    IntentPredictor: opponent destination + route (game knowledge, no ML)
    control/       HolonomicController (target -> velocity), DriverBlend (driver override), RulesGuard (pin limit)
    rules/         PinRules (game-manual numbers), PinTimer (contact time per opponent)
    manager/       DefenseAssist: tracker -> mode -> controller -> blend, once per 20 ms loop
                   TargetSelector: who to guard when focus is auto
  sim/             Simulator only. Never used on the robot.
    RobotBody      kinematic swerve with speed/accel limits
    CyclerBehavior opponent bot: pickup -> goal -> repeat (Cycler pushes through, Evader dodges,
                   reacting to where robots were 0.25 s ago like a human driver)
    Avoidance      Evader's steering around robots in its path
    Contact        pushing-match model: whoever drives harder into the other wins
    SimWorld       ground truth + collisions
    PerfectTracker OpponentTracker that reports exact truth
    NoisySensor    simulated camera: FOV, range, noise, latency, dropouts (feeds KalmanOpponentTracker)
    AnonymousSensor the same camera without ids: partners, color errors, clutter, occlusion
                   (feeds MultiTargetTracker); Scenarios can add two alliance partners
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
- [x] Human reaction delay for Evaders; walls, chokepoints and lanes; gate routing for both sides
- [x] Route prediction (IntentPredictor) and Cut-off mode
- [x] Anonymous detections (partners, color errors, clutter, occlusion) + multi-target tracker:
      see [docs/findings/anonymous-tracking.md](docs/findings/anonymous-tracking.md)
- [ ] Phase 7: real robot (AprilTag pose) with simulated ghost opponents
- [ ] Phase 8: vision-based opponent tracking

## Current findings

### Which mode when (best legal setup per situation, guarded opponent slowed, 180 s)

| Situation | Perfect tracking | Realistic camera |
| --- | --- | --- |
| Open field, opponent pushes through | **Cut-off + hold** (+47%) | **Cut-off + hold** (+31%) ≈ Line mirror + hold (+29%) |
| Open field, opponent dodges | **Line mirror + hold** (+43%) | **Line mirror + hold** (+19%) |
| One chokepoint, pushes through | **Line mirror + hold** (+44%) | **Cut-off + hold** (+47%) |
| One chokepoint, dodges | **Cut-off + hold** (+71%) | **Line mirror + hold** (+50%) ≈ Cut-off + hold (+47%) |
| Two lanes, either kind | **Line mirror + hold** (+41% / +48%) | not measured |
| Slow the whole alliance | Line mirror + hold, auto = threat (+13–26%) | not measured |

- **Line mirror + hold is the safe default.** It's never far from the best anywhere.
- **Cut-off is a specialist.** It predicts where the opponent will be and waits there. That's
  excellent at gaps and against robots that drive straight, but a dodger in open field sees it
  waiting and goes around (+13%). A version that switched to shadowing at close range was worse
  everywhere and was dropped.
- **Prediction helps with a noisy camera.** Cut-off depends on the route, not exact position, so
  it keeps more of its effect than Shadow (+31% vs +14% against bulldozers with a realistic camera).
- **WARNING, pins with a camera near walls:** with the realistic camera at the chokepoint, every
  mode lets through 1–5 pins per 3 minutes (0–1 in open field). The pin guard relies on camera
  estimates plus stall detection, and that's not enough near walls. **Real contact sensing on the
  robot is required before using hold in a match.**


### Details

Headless 180 s runs (`./gradlew run --args="--headless"`). Man-to-man modes guard opponent #0;
Zone picks its own threat. "No defense" parks our robot in a corner. **Slowed** = how much the
guarded opponent's average cycle time went up. **Pins** are counted like a referee would: only
contact time while we are driving into that opponent, one per episode that reaches 3 s.

### With perfect tracking

| Setup | vs Cyclers (push through) | vs Evaders (dodge) | Pins (C / E) |
| --- | --- | --- | --- |
| Shadow | +2% | +23% | 0 / 0 |
| **Shadow + hold** | **+24%** | **+32%** | **0 / 0** |
| Interpose + hold | +5% | +7% | 0 / 0 |
| Zone + hold | +4% | +1% | 0 / 0 |
| Line mirror | +35% | +42% | 0 / 0 |
| **Line mirror + hold** | **+36%** | **+43%** | **0 / 0** |
| Shadow + hold, no pin guard | +28% | jammed all match | 9 / 1 |
| Line mirror + hold, no pin guard | +50% | +50% | 17 / 4 |

### With a simulated camera (averaged over 5 seeds)

"Realistic" = two cameras (140°), 0.10 m noise, 80 ms latency, 15% dropped frames, 8 m range.
"Bad" = one 70° camera, 0.25 m noise, 150 ms, 35% dropped, 6 m. These are guesses until the real
pipeline is measured.

| Opponents | Setup | Perfect | Realistic | Bad |
| --- | --- | --- | --- | --- |
| Cyclers | Shadow + hold | +24% | +14% | +10% |
| Cyclers | Line mirror + hold | +36% | +29% | +12% |
| Evaders | Shadow + hold | +32% | +13% | +10% |
| Evaders | Line mirror + hold | +43% | +19% | +12% |

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
| Shadow + hold, auto = nearest | +4% | +8% |
| Shadow + hold, auto = threat | +7% | +7% |
| Line mirror + hold, locked on one scorer | +6% | +11% |
| Line mirror + hold, auto = nearest | +10% | +16% |
| Line mirror + hold, auto = threat | **+15%** | +13% |

- **Man-to-man (Shadow): pick one scorer and stay on them.** Switching costs more than it gains.
- **Positional defense (Line mirror): let it take whoever comes through** (auto = threat).
- Totals move by about ±1 cycle (≈ ±1.5%) between runs, so treat small gaps as ties.
- Oddity: plain Shadow does better with a noisy camera (+20%) than with perfect tracking (+2%).
  Tracking lag makes it drift into the opponent's path, which accidentally acts like hold.

### With walls (made-up layouts: one 2.5 m choke, or two 2 m lanes, 4 m in front of the goal)

Guarded opponent slowed, perfect tracking. Opponents route through gaps; Evaders pick a lane
without a defender near it. Baselines get slower with walls (Evaders: 7.15 s open, 8.5 s choke,
8.3 s two lanes), so these percentages are on top of that.

| Opponents | Setup | Open | Choke | Two lanes |
| --- | --- | --- | --- | --- |
| Cyclers | Shadow + hold | +24% | +15% | +22% |
| Cyclers | **Line mirror + hold** | +36% | **+44%** | **+41%** |
| Cyclers | Interpose + hold | +5% | +5% | +38% |
| Evaders | Shadow + hold | +32% | +16% | +21% |
| Evaders | **Line mirror + hold** | +43% | **+44%** | **+48%** |
| Evaders | Line mirror + hold, auto = threat | +7% | +17% | +44% |

- **Line mirror's lead survives walls and grows.** Its line sits just behind the gaps, so it
  covers every exit. Even with two lanes to choose from, Evaders can't get around it.
- **Shadow gets worse with walls** (+32% → +16–21% vs Evaders). Following an opponent through
  a narrow gap means arriving late at the exit.
- **Interpose finally works in two lanes** (+38% vs Cyclers), standing in the lane the opponent
  is committed to.
- With walls, 1 pin slipped through in 3 of 30 runs: the back-off sometimes points into a wall.
  A first attempt to fix that made things worse, so it's still open.

- Evaders react to a 0.25 s old view of other robots (human reaction time). With instant,
  superhuman reactions the Evader numbers were about 10–15 points lower.
- Limits: one defender, three opponents with fixed routes, no game pieces, idealized pushing,
  opponent identity assumed known. Treat rankings as direction, not guarantees.
