# Back-off for crowds (Phase 7 ghost sim and the simulator)

Branch `backoff-redesign`. Problem from [robot-pin-margin.md](robot-pin-margin.md): with three
ghosts the pin guard's back-off slid into a second ghost, then retreated straight along the
pusher's path until the geofence stopped it, and contact reached 2.3–14.4 s. Target: peak contact
at most 2.5 s with three ghosts, referee pins still 0, no gridlock. The margin rule from PR #15
(`RulesGuard.safetyMarginFor`) was not touched.

**Result: one ghost is fixed (24 of 24 setups at or under 2.5 s, was 22), the simulator's
two-lane gridlock is gone and pins fell. Three ghosts are not fixed: 10 of 24 setups at or under
2.5 s (was 8), three Cyclers still reach 5.1 s and one Evader run 11.1 s. The referee count is 0
everywhere.**

## What changed

`EscapePlanner` (formulas in its class comment, hand-computed tests in `CrowdEscapeTest`) and
`RulesGuard.backOff`:

1. **Every robot, where it will be** (idea 1, robots part). The planner predicts each robot over
   the look-ahead: a robot touching us keeps coming along its path at `max(|v|, our speed)`, any
   other tracked robot keeps its velocity, an untracked one stays put. Our own drive uses the
   robot's real back-off speed and acceleration (`setDriveLimits`), and the look-ahead is never
   shorter than one robot width: `D = max(v * 0.5 s, 0.95 m)` (1.5 m for the sim robot, 0.95 m at
   the 1 m/s Phase 7 caps). A direction that runs into a robot we are not touching is out.
2. **Never along a pusher's path; prefer more distance to everyone** (idea 2). A direction within
   45° of a touching robot's path is out (it would follow us at the same speed). Among the rest,
   the planner prefers leaving every threat's lane, then the largest predicted gap to every robot
   at the end of the look-ahead. It also never pushes on a robot it touches, threat or not.
3. **More ways out.** If left, right and back are all out, it tries both perpendiculars to every
   touching robot's path and 16 compass directions.
4. **Yield** (idea 3). If nothing is safe, it stands still with no feedforward: zero push effort
   toward every robot, so the referee counts nothing while we are pushed. If that lasts 1 s
   (`WEDGED_SECONDS`) it slides out at the shallowest angle past the threats (the old last resort,
   now over all 18+ directions). Without that slide the simulator gridlocked in 24 runs.
5. **Blocked directions** are ruled out by angle (within 30°) instead of by name.

`PinMarginStudy` now runs with program arguments (`./gradlew pinMarginStudy --args="..."`, no
`-D` properties), reports the referee's contact time next to the any-touch time, ghost slowed % and
gridlocks, and splits the contact time of long episodes by what the guard was doing.

## Results: `PinMarginStudy` (48 setups × 300 s), before → after

Peak = longest any-touch contact in one episode (what the pin guard counts, includes being
pushed). Referee = contact only while we drive into that ghost. 8 runs per row (0 / 100 ms
latency × Cyclers / Evaders × Line mirror / Cut-off + hold). Slowed = rise in the ghosts' mean
cycle time against our robot parked in a corner; it is coarse (6–30 cycles per run, one cycle is
3–17 points). Gridlocked = a ghost finished no cycle, or we touched ghosts for more than half the
run.

| Cap | Ghosts | Peak contact | Peak referee contact | Episodes > 2.5 s | Referee pins | Slowed | Gridlocked |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.5 m/s | 1 | 2.98 → **1.92 s** | 0.32 → 0.32 s | 6 → 0 | 0 → 0 | +6% → +4% | 0 → 0 of 8 |
| 0.5 m/s | 3 | 14.44 → 5.22 s | 0.34 → 1.10 s | 23 → 20 | 0 → 0 | −5% → −3% | 0 → 0 of 8 |
| 1.0 m/s | 1 | 2.40 → **2.40 s** | 1.18 → 1.18 s | 0 → 0 | 0 → 0 | +6% → +8% | 0 → 0 of 8 |
| 1.0 m/s | 3 | 7.26 → 11.10 s | 2.22 → 1.40 s | 32 → 24 | 0 → 0 | +6% → +5% | 0 → 0 of 8 |
| 4.5 m/s | 1 | 2.56 → **2.48 s** | 1.84 → 1.84 s | 1 → 0 | 0 → 0 | +22% → +22% | 0 → 0 of 8 |
| 4.5 m/s | 3 | 4.34 → 4.60 s | 1.84 → 1.92 s | 16 → 16 | 0 → 0 | +13% → +13% | 0 → 0 of 8 |

- **One ghost: never worse.** Per setup the peak went down or stayed (two rose by 0.02 s, far
  under 2.5 s). Slowed moved by one ghost cycle in three of 24 setups (0.5 m/s Evader, Line mirror:
  +17% → +0%, the same setup whose peak fell 2.98 → 0.40 s; 1.0 m/s Evader, Cut-off, both
  latencies: +0% → +8%).
- **Three ghosts: better on average, not fixed.** Episodes over 2.5 s 71 → 60. Evaders: 9 of 12
  setups at or under 2.5 s (was 7). Cyclers: still 1 of 12, peaks 2.3–5.1 s (was 2.3–7.3 s).
- **The referee is happy everywhere**: 0 pins, at most 1.92 s of contact while pressing.

Data: [before](data/backoff-study-before.txt), [after](data/backoff-study-after.txt), worst run
after: [timeline](data/backoff-worst-timeline-after.txt).

### Why three Cyclers are still over 2.5 s

Of the contact in the 60 long episodes: 50.6 s came before the first back-off (≈ the 1.18 s
trigger each), **137.2 s while backing off on an exit**, 15.8 s yielding, 1.4 s with the mode back
in charge. Only 10 of the 60 touched another ghost while over 2.5 s. So re-contact after the
back-off (idea 4's target) and yielding are small; the time goes into exits that are slower than
the margin assumes.

The margin is built for one clean move: at 1 m/s and 2 m/s² it backs off at 1.18 s, leaving 1.32 s
to slide one robot width sideways, which takes 1.2 s from rest plus 0.1 s latency. In a crowd the
clean perpendicular exit is often blocked (another ghost, or the field around the goal where all
three converge), and the next usable exit is further off the perpendicular (57° in one traced case). Its sideways speed is lower, so the slide
takes longer than the 0.12 s of slack. The back-off cannot make that up without starting earlier,
which is the margin rule (out of scope, and "margin from a predicted time-to-clear" already gave
no gain in PR #15). What would help is a mode-level change: not standing in the converging traffic
in front of the goal when two or more bulldozers are coming.

### Worst run after: 1.0 m/s, no latency, 3 Evaders, Cut-off + hold, 11.1 s

The robot backs off from an Evader that sits at the edge of its scoring circle, north of us, at
(11.6, 4.6). The chosen exit points east, which is free on the field, but the robot is on the
geofence line (x = 11.75), so `SafetyGate` stops it. The core does not know about the fence, so it
keeps the exit: robot and Evader stand touching for 10 s (we creep 0.02 m/s; referee time stays
0.92 s because we are not pressing). Every way of telling the planner about the fence made the
whole grid worse (below).

## Everything tried (same 48 setups; episodes over 2.5 s / peak in s)

Rows after "no geofence" change one thing from that row.

| Variant | 1 ghost | 3 ghosts | Referee peak / pins | Kept? |
| --- | --- | --- | --- | --- |
| Before (main) | 7 / 2.98 | 71 / 14.44 | 2.22 / 0 | — |
| Ideas 1–3, geofence as walls for the planner | 25 / 2.78 | 117 / 17.32 | 1.84 / 0 | no |
| Ideas 1–3, geofence as a fence we slide along | 2 / 2.62 | 76 / 6.42 | 1.84 / 0 | no |
| **Ideas 1–3, no geofence** (robots and field walls only) | **0 / 2.48** | **60 / 5.98** | 1.84 / 0 | base |
| … without the along-path rule (idea 2) | 7 / 2.98 | 77 / 10.66 | 1.84 / 0 | rule kept |
| … along-path at 30° | 31 / 2.56 | 53 / 5.70 | 1.84 / 0 | no (1 ghost) |
| … along-path at 60° | 2 / 116.9 | 66 / 29.72 | 1.84 / 0 | no (deadlock) |
| … without the 16 extra directions | 0 / 2.48 | 91 / 13.66 | 1.84 / 0 | extras kept |
| … without "prefer distance to everyone" | 0 / 2.48 | 61 / 6.00 | 1.84 / 0 | neutral, kept (idea 2) |
| … without "never run into another robot" | 0 / 2.48 | 61 / 8.82 | 1.84 / 0 | neutral, kept (idea 1) |
| … allowed to push on robots that are not threats | 0 / 2.48 | 57 / 7.44 | 1.84 / 0 | neutral; kept for the referee |
| … pushing allowed up to the referee's 0.1 effort | 15 / 2.54 | 54 / 6.38 | 1.84 / 0 | no |
| … re-engage only when clear of everyone by 0.5 m (idea 4) | 0 / 2.48 | 62 / 10.74 | 1.84 / 0 | no |
| … clear by 1.0 m (idea 4) | 0 / 2.48 | 67 / 7.12 | 1.84 / 0 | no |
| … re-engage only when the episode is over, 2.13 m (idea 4) | 0 / 2.50 | 60 / 18.32 | 1.84 / 0 | no |
| … distrust a pusher's velocity below half our speed | 25 / 3.04 | 70 / 6.96 | 1.84 / 0 | no |
| … look-ahead 1.0 s / 1.5 s of travel | 27 / 2.92, 53 / 2.92 | 58 / 6.66, 93 / 7.78 | 1.84 / 0 | no |
| … the old fixed 1.5 m look-ahead | 0 / 2.48 | 63 / 8.04 | 1.84 / 0 | neutral |
| … sim robot's speed in the predictions | 0 / 2.48 | 64 / 11.18 | 1.84 / 0 | neutral |
| Base (yield, no slide) in the simulator: **24 gridlocked runs** in `--batch-anon` (main: 2) | | | | no |
| + slide out after 1 s of yield, left / right / back only | 0 / 2.48 | 63 / 13.00 | 1.86 / 0 | no |
| **+ slide out after 1 s of yield, any direction** | **0 / 2.48** | **60 / 11.10** | 1.92 / 0 | **yes** (0 gridlocks) |
| that + geofence as walls | 25 / 2.78 | 99 / 9.96 | 7.42 / **4** | no |
| that + geofence slide | 2 / 2.62 | 65 / 8.98 | 5.06 / **2** | no |
| that + drop an exit that makes no progress for 0.5 s | 0 / 2.48 | 62 / 9.78 | 3.34 / **1** | no |

- **The geofence keeps hurting.** As walls the planner finds no exit near the fence and yields
  much more (contact while yielding in the long episodes 16 → 115 s); with the last-resort slide on, those slides press
  into ghosts against the fence and referee pins appear. This matches PR #15 ("geofence as walls":
  54 pins). The fence deadlock above is the price of leaving it out.
- **Idea 4 had nothing to fix**: contact after the mode takes over again is 1.4 s in total.
- **Yield alone gridlocks the simulator** (a bulldozer pushes a yielding robot into a wall and
  both stay there); the old 1 s slide-out is needed.
- Small changes still move the three-ghost numbers a lot in both directions (53–117 episodes),
  as PR #15 found, so read single setups as noisy.

## Cost in the simulator

- `--batch-anon 180 5` ([data](data/batch-anon.csv)): **gridlocked runs 2 → 0** (guarded opponent
  never finishes a cycle, or more than half the run in contact; the old two-lane Cut-off + hold
  gridlock). Pins, anonymous tracking, both cameras, Line mirror + hold (auto) and Cut-off + hold:
  **11 → 6 in 120 runs without partners, 10 → 9 with partners**; known ids 6 → 3 and 3 → 4. All
  rows: 42 → 34. Mean slowed of the grid +0.7 points (camera rows −0.5 to +1.4).
- `--headless` ([data](data/headless-integrated.txt)): pins 0 everywhere (Evader Shadow + hold with
  the Realistic camera: 0.4 → 0 per run). Perfect-tracking slowed moved both ways by up to ±9
  points without walls (Evaders: Shadow + hold +28% → +19%, Line mirror + hold +35% → +39%) and
  up to ±21 points with walls (Cut-off + hold vs Cyclers in two lanes +34% → +47%, Line mirror +
  hold vs Evaders in two lanes +45% → +66%, at the choke +42% → +30%). README "Current findings" is
  updated.

## How to run it

```bash
cd robot && ./gradlew pinMarginStudy --args="out=build/pin-margin-study"   # all 48, ~10 s
cd robot && ./gradlew pinMarginStudy --args="only=1.0_m/s___0_ms_Cycler_3"  # "_" = space
```

## Next

- Three Cyclers: a mode-level rule (step out of the converging lanes in front of the goal when two
  or more opponents that push through are coming) rather than a better back-off.
- The geofence: tell the planner about it without the last-resort slide pressing into ghosts at
  the fence (for example: no slide there, yield and let the ghost pass).
- Everything here is in the ghost sim with ideal contact; repeat on the robot with one ghost first
  ([phase7-test-plan.md](../phase7-test-plan.md)).
