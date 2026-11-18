# PhantomTrack

Driver-assist defense for FRC. The robot tracks opponents and helps the driver play different kinds
of defense (man-to-man, zone, lane blocking), switching between modes on the fly.

Right now this is a **simulator + algorithms**, plus a **robot project** (`robot/`, Phase 7) that runs
the same defense code on a real swerve robot against simulated "ghost" opponents. The sim can give
the defense exact poses, a noisy camera with known ids, or a realistic anonymous camera, so we can
develop and test defense logic before touching vision or a real opponent.

## Run it

You need Java 17 or newer. The JDK that comes with WPILib works.

```bash
./gradlew run                          # interactive sim window
./gradlew run --args="--headless"      # 180 s: no defense vs. every mode, both opponent types; saves snapshots/*.png
./gradlew run --args="--batch"         # every setup x opponents x sensor x 3 seeds -> results/batch.csv
./gradlew run --args="--batch 180 10"  # same, 180 s per run, 10 seeds
./gradlew run --args="--anonymous"     # anonymous camera vs known ids, partners on (see docs/findings/)
./gradlew run --args="--batch-anon 180 5"  # anonymous vs known ids, open/choke/two lanes, partners on/off -> results/batch-anon.csv
./gradlew run --args="--tracker-sweep" # how the multi-target tracker's settings were chosen
java -cp build/classes/java/main phantom.sim.ContactStudy 180 10 results/contact.csv  # pin-guard study (after ./gradlew classes)
./gradlew test                         # unit tests
(cd robot && ./gradlew build)          # robot project: compiles core + 8 ghost files from the sim, runs its tests
```

The robot project, its safety rules and the first-session test plan are in
[robot/README.md](robot/README.md) and [docs/phase7-test-plan.md](docs/phase7-test-plan.md).

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

**Pin guard (`RulesGuard`, always on)** times contact with each opponent. Contact comes from
`ContactDetector`, which fuses what the drivetrain feels (pushing but not moving, drive current,
sudden deceleration) with camera proximity, so it also works on robots the camera does not see.
Pin time belongs to a *contact episode*, not a track id, so it survives an anonymous tracker
renaming the robot we are pushing (`EpisodeKeys`). It backs off `safetyMarginFor` before the 3 s
placeholder limit, so that contact is over by 2.5 s even if the back-off has to slide a whole robot
width out of a head-on push:
`margin = 0.5 s slack + command latency + 20 ms loop + time to drive 0.95 m at min(speed cap, 3 m/s)
and the acceleration limit` (1.09 s for the sim robot, so it backs off at 1.91 s; 1.82 s at the
Phase 7 test caps of 1 m/s, 2 m/s² with an assumed 0.1 s latency, so it backs off at 1.18 s). It
backs off until they are 1.83 m apart, left, right or straight back, else in one of 16 other
directions (`EscapePlanner`): never into a wall, never pushing on a robot it touches, never
straight along a pusher's path (it would follow), never into another robot where that robot will
be over the look-ahead. If no direction is safe it **yields** (stands still, pushing on nobody),
and if that lasts 1 s it slides out at the shallowest angle. Limits
live in `PinRules` and must be updated from the 2027 manual. Every contact threshold is a
placeholder: calibrate it on the robot first ([procedure](docs/findings/contact-sensing.md#calibration-procedure-for-the-real-robot)).

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
    rules/         PinRules (game-manual numbers), PinTimer (contact time per episode), ContactDetector
                   (drivetrain + camera -> contact), DriveTelemetry, EscapePlanner (wall-aware back-off),
                   EpisodeKeys (pin time follows the contact episode when track ids change)
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
    DriveTelemetrySim, DriveMotorModel  commanded/measured velocity, acceleration, drive current
    ContactStudy   pin-guard study: pins, false-positive and missed contact time
    BatchRunner    every combination x seeds -> CSV for graphing
    ui/            Swing window + renderer
robot/             WPILib 2026 project (Phase 7). Compiles phantom/core and 8 AWT-free sim files
                   (robot/shared-sources.txt, checked by RobotSharedSourcesTest) straight from src/.
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
- [x] Contact sensing for the pin guard (drivetrain stall/current/deceleration + camera) and a
      wall-aware back-off: see [docs/findings/contact-sensing.md](docs/findings/contact-sensing.md)
- [x] Integration (Nov 16 work): pin time follows the contact episode when track ids change;
      pin-guard margin from the robot's back-off speed; `--batch-anon` grid
- [x] Phase 7 software: `robot/` project, ghost opponents against the real pose, safety rules
      (deadman, override, speed/accel caps, geofence, pose faults), AdvantageScope layout
- [x] Pin margin from back-off time + command latency (same rule in sim and robot), ghost-sim
      study tool `PinMarginStudy`: see [docs/findings/robot-pin-margin.md](docs/findings/robot-pin-margin.md)
- [x] Back-off for crowds (Nov 17): every robot and where it will be, never along a pusher's
      path, yield when there is no safe exit: see
      [docs/findings/backoff-redesign.md](docs/findings/backoff-redesign.md). One ghost now stays
      at or under 2.5 s in all 24 setups and the two-lane gridlock is gone
- [x] Audit (Nov 18): every formula re-derived, every hand-computed test value recomputed;
      fixes for NaN tracks, the red driver's sticks on mirrored fields, Shadow/Interpose chasing
      retreating opponents, rotation override, wall-attached obstacles, and more: see
      [docs/audit-2026-11.md](docs/audit-2026-11.md)
- [ ] Open from the audit: two Cyclers head-on inside a gap deadlock (sim, M5); hold ground
      pushes opponents that are leaving (U1, design question)
- [ ] Three ghosts: Cyclers still peak at 2.3–5.1 s of contact, one Evader run 11 s at the geofence
      (the referee count stays 0). With the fixed margin a crowded exit is too slow; needs a
      mode-level answer (stay out of the traffic) or the geofence in the planner without its
      last-resort slide pinning anyone
- [ ] Phase 7 on the robot: wire the drivetrain adapter, measure command latency, run
      [docs/phase7-test-plan.md](docs/phase7-test-plan.md) with one ghost
- [ ] Calibrate contact sensing on the robot (logs → replay through `ContactDetector`)
- [ ] Measure the real camera pipeline (noise, latency, color errors, false positives) and replace
      the Realistic/Bad guesses; mask repeating false positives
- [ ] Phase 8: vision-based opponent tracking (multi-camera fusion before `MultiTargetTracker`)
- [ ] 2027 game: `FieldConfig`, `PinRules`

## Current findings

All numbers come from 180 s simulator runs. Since the Nov 16 integration the pin guard always uses
contact sensing, and the camera column below is **anonymous tracking** (the camera reports
unlabeled boxes, a `MultiTargetTracker` turns them into tracks), which is what a real camera gives.
Camera rows are 5 seeds, ± is the standard error over seeds. Raw data (after the Nov 18 audit
fixes):
[docs/findings/data/audit-batch-anon-after.csv](docs/findings/data/audit-batch-anon-after.csv)
(`--batch-anon 180 5`) and [audit-headless-after.txt](docs/findings/data/audit-headless-after.txt)
(`--headless`); before them: [batch-anon.csv](docs/findings/data/batch-anon.csv),
[headless-integrated.txt](docs/findings/data/headless-integrated.txt).

### Which mode when (best legal setup per situation, guarded opponent slowed, 180 s)

| Situation | Perfect tracking | Realistic camera, anonymous tracking + contact sensing |
| --- | --- | --- |
| Open field, opponent pushes through | **Cut-off + hold** (+35%) | **Line mirror + hold** (+21% ± 1); Cut-off + hold +18% ± 1 |
| Open field, opponent dodges | **Line mirror + hold** (+39%) | **Line mirror + hold** (+15% ± 1); Cut-off + hold +12% ± 1 |
| One chokepoint, pushes through | Cut-off + hold (+34%) ≈ Line mirror + hold (+34%) | Line mirror + hold (+22% ± 1) ≈ Cut-off + hold (+21% ± 5) |
| One chokepoint, dodges | **Cut-off + hold** (+36%); Line mirror + hold +29% | Line mirror + hold (+35% ± 4) ≈ Cut-off + hold (+29% ± 3) |
| Two lanes, pushes through | **Cut-off + hold** (+47%); Line mirror + hold +27% | Line mirror + hold (+27% ± 2) ≈ Cut-off + hold (+25% ± 2) |
| Two lanes, dodges | **Line mirror + hold** (+54%) | **Line mirror + hold** (+34% ± 4); Cut-off + hold +27% ± 3 |
| Slow the whole alliance | Line mirror + hold, auto = threat (+10–24%) | Line mirror + hold, auto = threat (+8–16%) |

"Locked" man-to-man with anonymous tracks needs the driver to point the assist at the robot again
whenever its track is lost: the simulated driver does that 0.5 s after it notices, about 7.4 times
a minute (Realistic). Rows without alliance partners; partners are discussed below.

- **Line mirror + hold is the safe default, more than ever.** With a real (anonymous) camera it is
  best or tied in all 6 situations. It only needs one coordinate and does not care which robot is
  which.
- **Cut-off + hold still does not win anywhere with a real camera** (it ties in three situations).
  With perfect tracking it leads in three of six and ties Line mirror + hold in a fourth, but its
  lead does not survive the camera (open field vs bulldozers +35% → +18%). It no longer gridlocks
  in two lanes (0 of 5 runs, was 2).
- **A real camera costs about half the effect.** Bad camera (one 70° camera, 35% dropped frames):
  +4% to +15%, coverage of the opponents 18%. Camera coverage is the first thing to buy.
- **Anonymity mostly costs man-to-man.** Same camera with ids handed over vs anonymous: Line
  mirror + hold, auto = threat moves by 0–4 points, but Cut-off + hold locked on #0 drops (choke
  Evaders +43% → +29%, two lanes Evaders +33% → +27%, open Cyclers +21% → +18%).
- **Pins are not zero.** Anonymous tracking + contact sensing, both cameras, Line mirror + hold
  (auto) and Cut-off + hold, open/choke/two lanes, Cyclers and Evaders: **3 pins in 120 runs
  without partners (0.03 per 180 s run), 11 in 120 with partners (0.09 per run)** (6 and 9 before
  the Nov 18 audit fixes, 11 and 10 before the Nov 17 back-off redesign; counts this small move by
  a few between code changes that only alter the chaotic camera runs). Perfect tracking: 0. **Calibrate contact sensing on the robot
  before using hold in a match.**
- **The pin margin now depends on how fast the robot can back off** (Nov 17, see the next
  section). It backs off at 1.91 s of contact instead of 2.5 s in the sim. That roughly halved the
  pins above (from 16 and 22) and cost little with a camera (mean slowed of that grid −0.9 / −0.3
  points without / with partners), but up to 24 points with perfect tracking (Line mirror + hold vs
  Cyclers at the choke +51% → +27%).

### Pin guard with contact sensing

The guard used to time contact from camera estimates plus a stall check, and 1–5 pins per 3 min
slipped through near walls. It now fuses the drivetrain (stall, drive current, sudden
deceleration) with the camera, and backs off left / right / straight back with a wall check
([full write-up and calibration procedure](docs/findings/contact-sensing.md)). Known ids, no
partners, 10 seeds × Realistic/Bad × open/choke/two lanes × Cyclers/Evaders × Line mirror/Cut-off
+ hold (`ContactStudy 180 10`, 240 runs):

| | Before | After | Oracle (guard sees exact positions) |
| --- | --- | --- | --- |
| Pins | 338 (1.41 per run) | **29 (0.12 per run)** | 0 |
| False-positive contact time per run | 40.4 s | 15.2 s | — |
| Missed contact time per run | 4.8 s | 4.3 s | 0.3 s |

- **The remaining pins come from camera error near contact**: missed contact (10), episode closed
  too early (7), under-counted (5), still pressing during the back-off (6), other (1).
- **Being legal costs some slowed %** where the old numbers came from pinning (choke, Cut-off
  vs Cyclers, Realistic: +45.5% → +26.7%; the exact-contact "legal ceiling" is +31.4%).
- **Perfect tracking: pins with walls went 5 → 0**, the back-off no longer drives into walls. The
  perfect-tracking tables below moved a few points, mostly down (e.g. Line mirror + hold vs
  Cyclers +36% → +33%; Cut-off + hold vs Evaders at the choke +71% → +36%, the +71% was a lucky run).
- **Track ids that change (anonymous tracking).** Pin time used to be keyed by track id, so when
  the robot we were pushing came back under a new id, its timer restarted. Pin time now belongs to
  the contact episode (`EpisodeKeys`): a new id within 1 m of where a lost opponent was last seen
  continues its episode, and a new touch right after a lost one (0.5 s) inherits its time.
  Anonymous camera, partners on, open + choke, both setups, both cameras, 15 seeds (240 runs):
  **55 pins keyed by id → 40 with episodes** (each half alone: 52 and 47), for 6% more back-offs.
  With stable ids (perfect, known-id camera) nothing changes.
- **Robot speeds and command latency.** The guard backs off `safetyMarginFor` before the limit:
  `margin = 0.5 s slack + latency + 20 ms loop + time to drive 0.95 m (one robot width + 0.05 m) at
  min(cap, 3 m/s) and the acceleration limit`, the same rule in the sim and on the robot. Sim
  robot: 1.09 s. Phase 7 first test (1 m/s, 2 m/s², assumed 0.1 s latency): 1.82 s. In the
  Phase 7 ghost sim (`PinMarginStudy`, 48 setups × 300 s: 0.5 / 1 / 4.5 m/s, 0 / 100 ms extra
  latency, 1 / 3 ghosts, Cyclers / Evaders, Line mirror / Cut-off + hold), episodes with more than
  2.5 s of contact went **199 → 78**. With **one ghost** the peak is at most 2.5 s in 22 of 24 setups
  (2.98 s Evader at 0.5 m/s; 2.56 s at 4.5 m/s with 100 ms). With **three ghosts it is not fixed**:
  peaks of 2.9–14 s. Backing off earlier does not help there (0.9 s earlier: 78 → 97 episodes over
  2.5 s). The back-off itself fails: it slides into a second ghost, then retreats straight
  along the pusher's path, which follows at the same speed until the robot is held against the
  geofence. The referee count stayed 0 in every setup. Write-up and the worst timeline:
  [docs/findings/robot-pin-margin.md](docs/findings/robot-pin-margin.md).
- **Back-off for crowds (Nov 17).** The back-off now sees every robot and where it will be over the
  look-ahead (the robot's own speed and acceleration), never retreats along a pusher's path
  (within 45°), never pushes on a robot it touches, tries 16 more directions when left / right /
  back are all bad, and **yields** (stands still, zero push on everyone) when nothing is safe,
  sliding out after 1 s. `PinMarginStudy` (48 setups × 300 s), before → after:

  | Cap | Ghosts | Peak contact (any touch) | Peak referee contact | Episodes > 2.5 s | Referee pins | Slowed | Gridlocked |
  | --- | --- | --- | --- | --- | --- | --- | --- |
  | 0.5 m/s | 1 | 2.98 → **1.92 s** | 0.32 → 0.32 s | 6 → 0 | 0 → 0 | +6% → +4% | 0 → 0 of 8 |
  | 0.5 m/s | 3 | 14.44 → 5.22 s | 0.34 → 1.10 s | 23 → 20 | 0 → 0 | −5% → −3% | 0 → 0 of 8 |
  | 1.0 m/s | 1 | 2.40 → **2.40 s** | 1.18 → 1.18 s | 0 → 0 | 0 → 0 | +6% → +8% | 0 → 0 of 8 |
  | 1.0 m/s | 3 | 7.26 → 11.10 s | 2.22 → 1.40 s | 32 → 24 | 0 → 0 | +6% → +5% | 0 → 0 of 8 |
  | 4.5 m/s | 1 | 2.56 → **2.48 s** | 1.84 → 1.84 s | 1 → 0 | 0 → 0 | +22% → +22% | 0 → 0 of 8 |
  | 4.5 m/s | 3 | 4.34 → 4.60 s | 1.84 → 1.92 s | 16 → 16 | 0 → 0 | +13% → +13% | 0 → 0 of 8 |

  **One ghost is fixed** (all 24 setups at or under 2.5 s, was 22). **Three ghosts are not**: 10 of
  24 setups at or under 2.5 s (was 8). Evaders 9 of 12; Cyclers still peak at 2.3–5.1 s (was
  2.3–7.3 s); and one Evader run sits against the geofence for 11 s (the core does not know the
  fence; every way of telling it made things worse). The referee count is
  0 everywhere and the referee's own contact time stays at or under 1.92 s. In the simulator
  (`--batch-anon`) the two-lane gridlock is gone (2 → 0 runs) and pins fell (anonymous: 11 → 6
  without partners, 10 → 9 with). Write-up, every variant tried, and why three Cyclers are still
  over: [docs/findings/backoff-redesign.md](docs/findings/backoff-redesign.md).

### Anonymous tracking

Full write-up: [docs/findings/anonymous-tracking.md](docs/findings/anonymous-tracking.md). The
anonymous camera adds alliance partners, wrong bumper colors (5% / 15%), false detections (0.1 /
0.5 per frame), own-bumper detections and robots hidden behind robots (Realistic / Bad; all
guesses until the real pipeline is measured).

- **Positional defense barely notices; man-to-man costs driver attention** (see above). ID
  switches: about 30 per minute, against about 3 with ids handed over. Re-acquisition (a new track takes
  a lost id nearby, or the one missing id among 3 opponents) halves ID switches and re-points; a
  360° camera halves the re-points again.
- **Partners are seldom taken for opponents** (a fresh track sits on a partner about 3% of the
  time), but they matter physically: we touch our own partners about 20 s per run at the choke, and
  with partners on, Line mirror + hold (auto) vs Evaders at the choke drops from +20% to −1% ± 2.
  In two lanes the partners jam the Cyclers' lanes on their own (no-defense cycle 12.9 s vs 7.4 s
  without partners), so "slowed" there is negative and not meaningful.
- **Repeating false positives are the real danger** (experiment, off in the presets): with 3 fixed
  phantom spots, auto focus reacts to something that isn't a robot 18% of the time (vs 6%).
  Confirmation (3 of 5) only filters random clutter. A driver-chosen lock is mostly immune. Mask
  known field elements on the robot.

### Details

Headless 180 s runs (`./gradlew run --args="--headless"`), known ids, no partners, after the
Nov 17 back-off redesign (which moved these perfect-tracking numbers by up to ±9 points one way or
the other, more with walls; pins stayed 0) and the Nov 18 audit fixes
([docs/audit-2026-11.md](docs/audit-2026-11.md): Shadow and Interpose no longer chase an opponent
driving away from the goal into its bumpers, and hold ground's target is kept out of walls; pins
stayed 0). Man-to-man
modes guard opponent #0; Zone picks its own threat. "No defense" parks our robot in a corner.
**Slowed** = how much the guarded opponent's average cycle time went up. **Pins** are counted like
a referee would: only contact time while we are driving into that opponent, one per episode that
reaches 3 s.

### With perfect tracking

| Setup | vs Cyclers (push through) | vs Evaders (dodge) | Pins (C / E) |
| --- | --- | --- | --- |
| Shadow | +1% | +26% | 0 / 0 |
| **Shadow + hold** | **+15%** | **+28%** | **0 / 0** |
| Interpose + hold | +3% | +6% | 0 / 0 |
| Zone + hold | +6% | +4% | 0 / 0 |
| Line mirror | +28% | +38% | 0 / 0 |
| **Line mirror + hold** | **+26%** | **+39%** | **0 / 0** |
| Cut-off + hold | +35% | +13% | 0 / 0 |
| Shadow + hold, no pin guard | +27% | +136% (pinned most of the match) | 12 / 2 |
| Line mirror + hold, no pin guard | +50% | +50% | 17 / 4 |

### With a simulated camera, ids handed over (averaged over 5 seeds)

"Realistic" = two cameras (140°), 0.10 m noise, 80 ms latency, 15% dropped frames, 8 m range.
"Bad" = one 70° camera, 0.25 m noise, 150 ms, 35% dropped, 6 m. These are guesses until the real
pipeline is measured.

| Opponents | Setup | Perfect | Realistic | Bad |
| --- | --- | --- | --- | --- |
| Cyclers | Shadow + hold | +15% | +16% | +10% |
| Cyclers | Line mirror + hold | +26% | +23% | +12% |
| Evaders | Shadow + hold | +28% | +14% | +12% |
| Evaders | Line mirror + hold | +39% | +16% | +10% |

Pins with a camera: 0 per 180 s in this table (contact sensing; up to 0.4 before Nov 17). Position
estimate error: about 0.4 m (Realistic), 0.9–1.1 m (Bad). Time each opponent is tracked: 48–74%
(Realistic), 33–44% (Bad).

- **Holding ground matters most for man-to-man** (Shadow +1% → +15% vs Cyclers). Line mirror
  works with or without it (+28% / +26% vs Cyclers, +38% / +39% vs Evaders).
- **Camera coverage matters more than camera accuracy.** Going from Realistic to Bad roughly
  doubles the error, but the bigger loss comes from seeing opponents less often.

### Who to guard when nobody is chosen (effect on all three opponents' total cycles)

| Setup | vs Cyclers | vs Evaders |
| --- | --- | --- |
| Shadow + hold, locked on one scorer | +3% | +10% |
| Shadow + hold, auto = nearest | +4% | +10% |
| Shadow + hold, auto = threat | +12% | +7% |
| Line mirror + hold, locked on one scorer | +7% | +11% |
| Line mirror + hold, auto = nearest | **+10%** | **+15%** |
| Line mirror + hold, auto = threat | **+10%** | +13% |

- **Positional defense (Line mirror): let it take whoever comes through** (auto = threat or
  nearest): never worse than locking, and better vs Cyclers.
- **Man-to-man (Shadow): auto = threat looks best vs Cyclers** (+12% vs +3–4%), no clear winner
  vs Evaders (+7–10%). Totals move by about ±1 cycle (≈ ±1.5%) between runs, so treat small gaps
  as ties.
- Oddity: plain Shadow does better with a noisy camera than with perfect tracking. Tracking lag
  makes it drift into the opponent's path, which accidentally acts like hold.

### With walls (made-up layouts: one 2.5 m choke, or two 2 m lanes, 4 m in front of the goal)

Guarded opponent slowed, perfect tracking. Opponents route through gaps; Evaders pick a lane
without a defender near it. Baselines get slower with walls (Evaders: 7.15 s open, 8.5 s choke,
8.3 s two lanes), so these percentages are on top of that.

| Opponents | Setup | Open | Choke | Two lanes |
| --- | --- | --- | --- | --- |
| Cyclers | Shadow + hold | +15% | +18% | +23% |
| Cyclers | Line mirror + hold | +26% | **+34%** | +27% |
| Cyclers | Interpose + hold | +3% | +8% | +30% |
| Cyclers | **Cut-off + hold** | **+35%** | **+34%** | **+47%** |
| Evaders | Shadow + hold | +28% | +19% | +28% |
| Evaders | **Line mirror + hold** | **+39%** | +29% | **+54%** |
| Evaders | Line mirror + hold, auto = threat | +12% | +24% | +32% |
| Evaders | Cut-off + hold | +13% | **+36%** | +19% |

- **With perfect tracking, Cut-off or Line mirror is at the top everywhere.** Cut-off + hold
  leads against bulldozers in open field and two lanes, ties Line mirror + hold at the choke
  (+34% each) and leads against dodgers at the choke; Line mirror + hold leads against dodgers in
  open field and two lanes, where they can't get around it (+54%). These
  single runs moved by up to ±21 points with the Nov 17 back-off redesign, so treat gaps under ~10
  points as ties. With a camera, Line mirror stays the safe default (see the top table).
- **Interpose works in two lanes** (+30% vs Cyclers), standing in the lane the opponent is
  committed to.
- Pins with walls and perfect tracking: 0 (5 before contact sensing).

- Evaders react to a 0.25 s old view of other robots (human reaction time). With instant,
  superhuman reactions the Evader numbers were about 10–15 points lower.
- Limits: one defender, three opponents with fixed routes, no game pieces, idealized pushing, a
  made-up camera and partner layout, and every contact threshold tuned on the same simulator.
  Treat rankings as direction, not guarantees.
