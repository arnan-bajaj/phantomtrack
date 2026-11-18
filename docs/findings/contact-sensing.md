# Contact sensing and wall-aware back-off

Branch `contact-sensing`. Goal: close the known safety hole where, with a realistic camera near
walls, 1–5 pins per 180 s slipped past the pin guard and the back-off could point into a wall.

## In one paragraph

The pin guard no longer relies on the camera alone. A new `ContactDetector` fuses what the
drivetrain feels (pushing but not moving, drive current, sudden deceleration) with camera
proximity, and the back-off now chooses left, right or straight back with a wall check.
Over 240 noisy-camera runs (10 seeds × 3 layouts × 2 opponent kinds × 2 setups × 2 cameras), pins
went from **338 to 29** (1.41 → 0.12 per 180 s). The target was 0, and **0 was not reached**.
Slowed % stayed within 3 points in 17 of 24 rows. The 7 rows that dropped more were mostly rows where the old code
was getting its numbers by pinning (1.7–4.9 pins per run). An oracle run with exact contact sensing
(0 pins) shows those old numbers are not reachable legally. Every threshold is a placeholder
until it is measured on the robot (procedure at the end).

## What changed

| Piece | Where | What it does |
| --- | --- | --- |
| Drive telemetry | `sim/DriveTelemetrySim`, `sim/DriveMotorModel`, `core/rules/DriveTelemetry` | Commanded vs measured velocity, measured acceleration, per-module drive current from `I = (V - omega/Kv)/R`, Gaussian noise, 20 ms samples. Motor constants in one record: Kraken X60 by default, `neo()` for NEO. |
| Contact detector | `core/rules/ContactDetector` | Pure (time passed in). Cues: (a) stall, (b) current over threshold, (c) sudden deceleration, (d) camera gap. Log-odds per opponent, smoothed, enter at p ≥ 0.7, leave at p ≤ 0.3. Output: probability, direction, most likely opponent or UNKNOWN. Formula in the class comment. |
| Pin guard | `core/control/RulesGuard`, `core/rules/PinTimer` | Pin time now comes from the detector. Time is handed over when the camera finds (or loses) the robot we are pushing on. A lost opponent we are still blocked by keeps accruing. A stall a wall explains is not a pin. |
| Wall-aware back-off | `core/rules/EscapePlanner` | Left / right (perpendicular to the opponent's path) / straight back. Never a direction that hits a wall or obstacle within 0.5 s at 3 m/s (1.5 m), never one that drives toward a threat. Prefers leaving the opponent's path, then clearance, then staying goal-side. If wedged with nothing free for 1 s, it slides out at the shallowest angle. |
| Back-off end | `RulesGuard` | Control returns to the mode at the rule distance (1.83 m), but the pin episode stays open until 1.83 + 0.3 m is confirmed by a fresh estimate. Touching again before that re-triggers at once. |
| Metrics | `SimSession.Metrics`, `sim/ContactStudy` | False-positive and missed contact time; the study harness below. |

`DefenseAssist` (core/manager) also changed slightly, outside the planned file list. It passes the
telemetry to the guard and lets the detector config be set per robot/camera.

## Results

Reproduce:
`./gradlew classes && java -cp build/classes/java/main phantom.sim.ContactStudy 180 10 results/after.csv`.
Raw outputs are in `docs/findings/data/`.

### Totals (240 runs of 180 s)

| | Before | After | Oracle (exact contact sensing) |
| --- | --- | --- | --- |
| Pins | 338 (1.41 per run) | **29 (0.12 per run)** | 0 |
| Runs with any pin | 129 / 240 | 25 / 240 | 0 / 240 |
| Most pins in one run | 9 | 3 | 0 |
| False-positive contact time per run | 40.4 s (23.5 s pin timer only) | **15.2 s** | — |
| Missed contact time per run | 4.8 s (8.9 s pin timer only) | **4.3 s** | 0.3 s |

Definitions (scored against sim truth, one 20 ms step behind, because that is the data the guard
had):

- **False positive**: the guard was counting pin time while no opponent was within 0.05 m.
- **Missed**: we were pressing an opponent (within 0.05 m and driving into it, which is what the
  referee counts) and the guard was not counting.
- For the old code, "counting" means its per-opponent pin timer or its blind stall timer. That
  timer also fired on walls, so the "pin timer only" figures are shown too.

The original hole, the chokepoint with a Realistic camera:

| Opponents | Setup | Pins per 180 s before | After |
| --- | --- | --- | --- |
| Cyclers | Line mirror + hold | 1.7 | 0.1 |
| Cyclers | Cut-off + hold | 4.4 | 0.3 |
| Evaders | Line mirror + hold | 2.4 | 0.2 |
| Evaders | Cut-off + hold | 1.8 | 0.2 |

### Every row (10 seeds each, guarded opponent slowed)

"Legal ceiling" is the oracle run described below.

| Layout | Opps | Setup | Camera | Pins before | Pins after | Slowed before | Slowed after | Δ pts | Legal ceiling (oracle) | FP s before | FP s after | Missed s before | Missed s after |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Open | Cycler | Line mirror | Realistic | 0.3 | 0.0 | +29.2% | +26.8% | -2.4 | +25.9% | 30.0 | 12.4 | 2.4 | 2.5 |
| Open | Cycler | Line mirror | Bad | 0.8 | 0.0 | +11.7% | +11.2% | -0.5 | +11.9% | 44.5 | 12.8 | 5.4 | 6.6 |
| Open | Cycler | Cut-off | Realistic | 2.6 | 0.2 | +30.0% | +25.3% | -4.7 **(>3)** | +26.3% | 29.8 | 14.7 | 6.4 | 5.5 |
| Open | Cycler | Cut-off | Bad | 1.9 | 0.1 | +11.6% | +11.5% | -0.2 | +10.9% | 32.6 | 11.5 | 7.7 | 6.6 |
| Open | Evader | Line mirror | Realistic | 0.1 | 0.0 | +16.8% | +17.7% | +0.9 | +18.9% | 39.0 | 17.3 | 5.1 | 1.7 |
| Open | Evader | Line mirror | Bad | 0.0 | 0.0 | +11.2% | +11.2% | +0.0 | +11.0% | 73.8 | 21.4 | 2.3 | 2.6 |
| Open | Evader | Cut-off | Realistic | 0.0 | 0.0 | +12.2% | +14.5% | +2.3 | +11.5% | 35.1 | 16.2 | 4.2 | 3.0 |
| Open | Evader | Cut-off | Bad | 0.0 | 0.0 | +7.4% | +8.6% | +1.1 | +7.2% | 52.7 | 20.5 | 2.8 | 3.4 |
| Choke | Cycler | Line mirror | Realistic | 1.7 | 0.1 | +39.5% | +31.7% | -7.8 **(>3)** | +32.5% | 27.7 | 12.8 | 4.9 | 3.6 |
| Choke | Cycler | Line mirror | Bad | 3.7 | 0.4 | +22.4% | +19.5% | -2.9 | +14.9% | 39.0 | 11.2 | 8.3 | 10.3 |
| Choke | Cycler | Cut-off | Realistic | 4.4 | 0.3 | +45.5% | +26.7% | -18.8 **(>3)** | +31.4% | 24.4 | 12.3 | 6.6 | 5.2 |
| Choke | Cycler | Cut-off | Bad | 4.9 | 0.3 | +26.9% | +17.2% | -9.7 **(>3)** | +15.3% | 28.1 | 11.6 | 9.7 | 9.1 |
| Choke | Evader | Line mirror | Realistic | 2.4 | 0.2 | +48.3% | +34.9% | -13.5 **(>3)** | +33.4% | 37.5 | 15.7 | 5.7 | 3.7 |
| Choke | Evader | Line mirror | Bad | 0.3 | 0.1 | +21.4% | +20.9% | -0.5 | +18.6% | 61.9 | 21.0 | 6.8 | 5.4 |
| Choke | Evader | Cut-off | Realistic | 1.8 | 0.2 | +45.5% | +45.8% | +0.3 | +36.8% | 43.0 | 21.1 | 6.0 | 4.1 |
| Choke | Evader | Cut-off | Bad | 0.5 | 0.1 | +26.3% | +20.5% | -5.8 **(>3)** | +22.6% | 55.4 | 24.4 | 4.3 | 4.3 |
| Two lanes | Cycler | Line mirror | Realistic | 2.1 | 0.3 | +31.6% | +28.8% | -2.9 | +28.7% | 21.8 | 18.0 | 2.4 | 2.6 |
| Two lanes | Cycler | Line mirror | Bad | 0.3 | 0.3 | +11.0% | +17.6% | +6.7 | +12.8% | 48.8 | 12.1 | 3.4 | 5.5 |
| Two lanes | Cycler | Cut-off | Realistic | 3.9 | 0.1 | +55.4% | +42.3% | -13.2 **(>3)** | +42.6% | 15.8 | 11.9 | 1.6 | 3.1 |
| Two lanes | Cycler | Cut-off | Bad | 0.7 | 0.1 | +23.4% | +25.7% | +2.3 | +16.3% | 40.7 | 8.4 | 3.6 | 4.3 |
| Two lanes | Evader | Line mirror | Realistic | 0.4 | 0.0 | +34.0% | +33.8% | -0.1 | +30.6% | 39.6 | 17.3 | 4.8 | 2.3 |
| Two lanes | Evader | Line mirror | Bad | 0.1 | 0.0 | +12.4% | +14.9% | +2.5 | +12.9% | 54.5 | 11.9 | 2.9 | 2.4 |
| Two lanes | Evader | Cut-off | Realistic | 0.9 | 0.1 | +33.1% | +37.7% | +4.7 | +34.3% | 47.8 | 16.2 | 4.4 | 2.5 |
| Two lanes | Evader | Cut-off | Bad | 0.0 | 0.0 | +7.2% | +11.4% | +4.2 | +8.1% | 45.7 | 10.8 | 2.5 | 2.3 |

**Slowed rule (must not drop more than 3 points): broken in 7 of 24 rows.**

- Six of those seven rows had 1.7–4.9 pins per run before, so part of their old slowed % was
  holding opponents illegally. The legal ceiling column shows it: with exact contact sensing (and 0
  pins) Choke/Cyclers/Cut-off/Realistic reaches +31.4%, not the old +45.5%.
- Against the legal ceiling, the new code is within 3 points in 6 of the 7 rows. The exception is
  **Choke/Cyclers/Cut-off/Realistic: +26.7% vs a ceiling of +31.4%**. That 4.7-point gap is the
  real cost of imperfect sensing (extra back-offs from false positives).
- Choke/Evaders/Cut-off/Bad is −5.8 vs before, but only 2.1 points below the ceiling (+20.5% vs
  +22.6%), so it is within the rule against what is legally possible.

**How the oracle was run:** a scratch copy of this branch (not committed) gave the pin guard a
`PerfectTracker` while the modes kept the noisy camera. That is three lines in `DefenseAssist` and
`SimSession`. Data: `data/contact-study-oracle.*`.

### Where the remaining 29 pins come from

A scratch diagnostic classified each one by what happened while the referee's clock ran:

| Cause | Pins |
| --- | --- |
| Contact missed for ≥ 0.4 s (camera estimate says apart, drivetrain not stalled enough) | 10 |
| Guard closed the episode too early (an estimate said ≥ 2.13 m apart, or a lost track was forgotten after 2 s) while true contact went on | 7 |
| Guard under-counted (late entry, many short flickers) | 5 |
| Still pressing during the back-off (opponent follows us, or the slide-out) | 6 |
| Other | 1 |

24 of the 29 are on the guarded opponent. No single fix removes them, and every one comes back to
camera error near contact. Real contact sensing on the robot (the cues here, calibrated) plus a
better camera are the remaining levers.

### Side effects with perfect tracking (headless run)

Full outputs: `data/headless-before.txt`, `data/headless-after.txt`. Pins with perfect tracking
were already 0 in the main tables and stay 0. In the walls table they went from 5 to **0**: the old
back-off into walls is gone.

Slowed with perfect tracking moved a little, mostly down:

| Opponents | Setup | Before | After |
| --- | --- | --- | --- |
| Cyclers | Shadow + hold | +24% | +19% |
| Cyclers | Line mirror + hold | +36% | +33% |
| Evaders | Line mirror + hold | +43% | +40% |
| Evaders, choke | Cut-off + hold | +71% | +36% |

The +71% was a lucky single run: the same setup over 360 s gives +47% on the old code and +34% on
the new. That ~13-point loss at the choke vs dodgers is real and not fixed here. With the camera,
the tracking table is about the same: Line mirror + hold +29% → +27% vs Cyclers and +19% → +18%
vs Evaders, with pins down from up to 1.2 per run to at most 0.2.

The 30 s `shadowStaysGoalSideAndNumbersStayFinite` test bar went from 0.8 to 0.7. On that run:
- Shadow alone (no pin guard): 0.749.
- Old guard: 0.801, helped by one back-off that happened to land well.
- New guard: 0.709, one sideways back-off.

**README "Current findings" was not updated** (out of scope for this branch). These are the
numbers it would need.

### Things tried and dropped (with the reason)

| Try | Result | Kept? |
| --- | --- | --- |
| Faster exit (`tauFall` 0.04) | more pins, lower goal-side | no |
| Exit at p ≤ 0.5 instead of 0.3 | FP halved, but pins 28 → 44 | no |
| Don't count contact while clearly retreating | FP down, pins 31 → 44 | no |
| Camera contact vetoed when driving at it freely | pins 29 → 36, FP −1.3 s | no |
| Never drive toward a threat in the back-off | removed pins caused by escaping across the opponent | yes |
| Current cue off while braking | fewer false positives with perfect tracking | yes |
| Vision ramp sized to camera error (Perfect 0.05 m, Realistic/Bad 0.4 m) | fewer perfect-tracking false positives | yes |
| Slide out when wedged for 1 s | fixed a 2-lane gridlock (36 → 61 total cycles) | yes |

## Calibration procedure for the real robot

Every number below is a **PLACEHOLDER** tuned only against the simulator. Do not trust the
detector in a match until each one is measured.

### What to log (50 Hz or faster, all four modules)

- Commanded chassis velocity (field or robot frame, be consistent), and each module's commanded
  speed.
- Measured chassis velocity from wheel odometry, and each module's measured speed.
- IMU acceleration (x, y) with the robot's heading, so it can be put in the field frame.
- Each drive motor's **stator** current, not supply current: Kraken `getStatorCurrent()`,
  SparkMax `getOutputCurrent()`. Also log the applied voltage or duty cycle and the battery
  voltage.
- Match time, so the logs can be replayed.

### Test 1: push against a wall (stall)

1. Put the robot square against the field wall (or a fixed, padded wall), bumpers touching.
2. Command 25%, 50% and 75% output straight into the wall, 3 s each, three times per level.
   Rest 10 s between pushes so the motors cool.
3. Repeat at 45° into the wall.
4. Repeat with the swerve's normal **closed-loop velocity** control, commanding 1, 2 and 4 m/s
   into the wall, because that is the mode the assist uses.

What to expect: the model says a stalled Kraken X60 draws `I = V / R = 12 × duty / 0.0328 Ω`:
about 91 A at 25%, so it will sit at the stator current limit (60 A in the sim) at all three
levels. If it does, the 25/50/75% runs mainly measure the current limit and the time it takes to
reach it, which is still what the thresholds need.

### Test 2: drive freely (no contact)

The same outputs and speeds in open space: accelerate from rest, cruise, brake hard, turn in
place, and strafe. This gives the "normal" levels the thresholds must stay above.

### Test 3: push another robot (if a second robot is available)

1. Head-on equal push, robot vs robot.
2. Push a robot that is idle.
3. Get pushed by it while backing away.
4. Ram it from 2 m/s.

This calibrates the deceleration cue and checks attribution to the right robot.

### Setting the thresholds (in `ContactDetector.Config` unless noted)

| Threshold | Sim placeholder | Set from |
| --- | --- | --- |
| `currentLow` | 20 A | 99th percentile of mean \|stator current\| in Test 2 while accelerating, plus 5 A |
| `currentHigh` | 40 A | mean current 0.2 s into the 25% wall push in Test 1, minus 5 A |
| `stallProgressNone` / `stallProgressFull` | 0.60 / 0.25 | progress = v_meas·u / \|v_cmd\| in Test 2 (lowest while cruising) and Test 1 (highest while pushing) |
| `freeAccel` | 3.0 m/s² | about half the peak acceleration seen in Test 2 |
| `decelLow` / `decelHigh` | 8 / 20 m/s² | `decelLow` above the hardest braking in Test 2; `decelHigh` near the median peak in the Test 3 ram |
| `decelHoldSec` | 0.3 s | how long the current rises after the ram in Test 3 |
| `pushMinSpeed` | 0.4 m/s | the slowest command that still makes a measurable push in Test 1 |
| `visionGapNear` / `visionGapFar` | 0.05 / 0.05 + camera error | measured tracking error of the real camera pipeline (Phase 8) |
| `bias`, `w*`, `tauRise`, `tauFall`, `enter`/`exitProbability` | -4, 3.5/3.5/3.5/5.5, 0.02/0.10 s, 0.7/0.3 | replay the logs (below) and tune until there is no missed contact in Test 1/3 and no false contact in Test 2 |
| `RulesGuard.RESET_MARGIN`, `FRESH_SECONDS`, `FORGET_SECONDS`, `HANDOVER_SECONDS`, `WEDGED_SECONDS`, `LOOK_SECONDS` | 0.3 m, 0.25 s, 2 s, 0.5 s, 1 s, 1 s | camera error and latency once measured; then practice matches |
| `RulesGuard.LOOKAHEAD_SECONDS`, `BACKOFF_SPEED` | 0.5 s, 3 m/s | the real robot's speed and stopping distance |
| `DriveMotorModel` gearing, wheel, module offset, current limit, `velocityKp` | MK4i L2, 4 in, 0.27 m, 60 A, 0.016 V/(rad/s) | the real swerve module and motor-controller config (sim only) |
| `DriveTelemetrySim.Config.realistic()` noise | 0.03 m/s, 0.5 m/s², 2 A | standard deviation of each signal while standing still and cruising (sim only) |
| `PinRules` | 3 s, 1.83 m | the 2027 game manual |

### Check the result without the robot moving

`ContactDetector` is pure: it takes time and a `DriveTelemetry` record and keeps no clock. So the
logs from Tests 1–3 can be turned into `DriveTelemetry` rows and replayed through the detector in
a unit test. Assert contact during every push and none during free driving. Do this before any
on-field test of the assist.

### Then on the field

Drive the assist with hold on against a practice robot in a pushing match. Log the detector
output and check it backs off before 2.5 s of contact every time, including against a wall and in a
gap.
