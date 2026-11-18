# Pin margin on the robot (Phase 7 ghost sim)

Branch `robot-pin-margin`. Problem from PR #14: against one 1 m/s ghost the pin guard's contact
timer peaked at exactly 3.00 s (no slack), and with three ghosts it passed 3 s at every speed.
Real drivetrain latency would make it worse. Target: peak contact at most 2.5 s (0.5 s of real
slack).

**Result: fixed for one ghost (22 of 24 setups at or under 2.5 s, with and without 100 ms
latency). Not fixed for three ghosts: that needs a back-off redesign, not a better margin.**

## How it was measured

`robot/src/test/java/frc/robot/defense/PinMarginStudy.java` runs the real robot pipeline
(`PhantomTrack`: assist, VirtualContact, SafetyGate with the geofence) against ghosts for 300 s,
with a perfect drivetrain that applies each command 0 or 100 ms late. Grid: speed cap 0.5 / 1.0
m/s (2 m/s²) and 4.5 m/s (6 m/s², the sim robot) × latency 0 / 100 ms × Cyclers / Evaders × 1 / 3
ghosts (ghost top speed = our cap) × Line mirror + hold / Cut-off + hold, first-test area (5 × 5 m,
0.75 m fence margin). 48 runs.

```bash
cd robot && ./gradlew pinMarginStudy --args="out=build/study"   # program arguments since Nov 17
```

"Contact time" is GhostSession's any-touch timer: per ghost, time with a bumper gap of at most
0.05 m, summed over an episode that only ends at 1.83 m apart. It counts being pushed too, so it
is stricter than the referee (who counts only contact while we press in). The referee count is
reported too, and it was **0 in every run, before and after**.

Each episode that passed 2.5 s was classified: LATE (already over 2.5 s when its first back-off
started), CHASED (most of the extra time was during a back-off with the ghost still touching),
INTO_OTHER (the back-off touched another ghost), RECONTACT (a new touch after the back-off ended),
OTHER.

## Why three ghosts break it

Before (old margin 1.17 s at 1 m/s, 0.5 s at 4.5 m/s): 199 episodes over 2.5 s. By class: chased
122, into another ghost 22, late 52 (all at 4.5 m/s), recontact 0, other 3. After: chased 63, into
another ghost 14, late 0, recontact 0, other 1.
Candidates from the brief:

- **Several opponents' timers running at once: no.** "Late" episodes (already past 2.5 s when the
  back-off started) all come from the old 0.5 s margin at 4.5 m/s, which leaves no time to get out.
  The new margin removes all of them.
- **Backing off into another ghost: yes, as the start of the worst runs.**
- **Re-contact right after a back-off: no** (0 episodes).
- **What dominates: the back-off does not get out.** It slides into a second ghost, is ruled out,
  and then retreats straight along the pusher's path. The pusher drives at the same speed, so it
  stays in contact until the robot reaches the geofence and stops. The back-off direction
  (`EscapePlanner`) does not know about other robots as obstacles, about where an attacker is
  going, or about the geofence.

### Worst run, before: 1.0 m/s, no latency, 3 Cyclers, Cut-off + hold, peak 8.0 s

Every 0.5 s (`gN gap` = bumper gap to ghost N, `t` = its contact time). Full log:
[data/pin-margin-worst-timeline-before.txt](data/pin-margin-worst-timeline-before.txt).

| t (s) | State | Us (x, y) | g0 gap / t | g1 gap / t | g2 gap / t | What happens |
| --- | --- | --- | --- | --- | --- | --- |
| 277.28 | ENGAGED | (8.34, 4.03) | 1.45 / 0.34 | 1.07 / 0 | 0.00 / 0.20 | Cycler g2 runs into us (hold: we push back) |
| 278.28 | ENGAGED | (8.27, 4.04) | 0.76 / 0.34 | 0.44 / 0 | 0.00 / 1.02 | stalled head-on; g0 and g1 close in on both sides |
| 279.28 | BACKING_OFF | (8.29, 3.89) | 0.23 / 0.34 | 0.44 / 0 | 0.00 / 2.02 | guard at 1.83 s: backs off sideways (-y) |
| 279.78 | BACKING_OFF | (8.32, 3.48) | **0.01** / 0.60 | 1.05 / 0 | 0.01 / 2.52 | slid into g0; that side is ruled out |
| 280.28 | BACKING_OFF | (8.45, 3.30) | 0.24 / 0.70 | 1.57 / 0 | 0.01 / 3.02 | new direction: straight back, along g2's path to the goal |
| 281.28 | BACKING_OFF | (9.38, 3.41) | 1.99 / 0 | 2.96 / 0 | 0.01 / 4.02 | full speed (1 m/s); g2 follows at 1 m/s, still touching |
| 282.78 | BACKING_OFF | (10.87, 3.63) | | | 0.01 / 5.52 | |
| 283.78 | BACKING_OFF | (11.73, 3.78) | | | 0.00 / 6.52 | reaches the geofence (x = 11.75) and stops |
| 284.78 | BACKING_OFF | (11.75, 3.93) | | | -0.00 / 7.52 | held against the fence by g2 heading for the goal (13, 4.035) |

## The margin rule (same in the sim and on the robot)

`RulesGuard.safetyMarginFor` (formula in its Javadoc, hand-computed tests in `IdSwitchPinTest`):

```
v      = min(speed cap, BACKOFF_SPEED = 3 m/s)
d      = 2 * robotHalfSize + CONTACT_TOLERANCE = 0.95 m   (slide one robot width out of a head-on push)
T(d)   = sqrt(2 d / a)                  if d <= v^2 / (2a)
       = v / a + (d - v^2 / (2a)) / v   otherwise
margin = 0.5 s slack + command latency + 0.02 s loop + T(d), at most the limit
```

| Robot | v, a, latency | T(d) | Margin | Backs off at (3 s limit) |
| --- | --- | --- | --- | --- |
| Simulator | 3 m/s (of 4.5), 6 m/s², 0 | 0.567 s | 1.087 s | 1.91 s |
| Phase 7 first test | 1 m/s, 2 m/s², 0.1 s (assumed) | 1.2 s | 1.82 s | 1.18 s |
| Slow test | 0.5 m/s, 2 m/s², 0.1 s | 2.025 s | 2.645 s | 0.36 s |

The robot's latency is `SafetyConfig.ASSUMED_COMMAND_LATENCY_SEC` = 0.1 s, a placeholder: measure
it from a command step in the log. The study's 0 ms runs also use the assumed 0.1 s (the robot
does not know its real latency).

## Peak contact per setup (s), before → after

| Cap | Extra latency | Ghosts | n | Setup | Before | After |
| --- | --- | --- | --- | --- | --- | --- |
| 0.5 m/s | 0 ms | Cycler | 1 | LM+hold | 3.30 | 1.92 |
| 0.5 m/s | 0 ms | Cycler | 1 | CO+hold | 2.18 | 0.64 |
| 0.5 m/s | 0 ms | Cycler | 3 | LM+hold | 4.90 | 5.80 |
| 0.5 m/s | 0 ms | Cycler | 3 | CO+hold | 6.54 | 5.30 |
| 0.5 m/s | 0 ms | Evader | 1 | LM+hold | 1.06 | 2.98 |
| 0.5 m/s | 0 ms | Evader | 1 | CO+hold | 1.12 | 0.42 |
| 0.5 m/s | 0 ms | Evader | 3 | LM+hold | 2.72 | 2.86 |
| 0.5 m/s | 0 ms | Evader | 3 | CO+hold | 3.20 | 1.82 |
| 0.5 m/s | 100 ms | Cycler | 1 | LM+hold | 1.54 | 0.36 |
| 0.5 m/s | 100 ms | Cycler | 1 | CO+hold | 2.32 | 1.20 |
| 0.5 m/s | 100 ms | Cycler | 3 | LM+hold | 2.60 | 6.72 |
| 0.5 m/s | 100 ms | Cycler | 3 | CO+hold | 5.90 | 5.52 |
| 0.5 m/s | 100 ms | Evader | 1 | LM+hold | 0.76 | 0.38 |
| 0.5 m/s | 100 ms | Evader | 1 | CO+hold | 2.04 | 0.42 |
| 0.5 m/s | 100 ms | Evader | 3 | LM+hold | 1.26 | 14.44 |
| 0.5 m/s | 100 ms | Evader | 3 | CO+hold | 1.22 | 2.94 |
| 1.0 m/s | 0 ms | Cycler | 1 | LM+hold | 2.16 | 1.54 |
| 1.0 m/s | 0 ms | Cycler | 1 | CO+hold | 3.00 | 2.32 |
| 1.0 m/s | 0 ms | Cycler | 3 | LM+hold | 4.86 | 3.60 |
| 1.0 m/s | 0 ms | Cycler | 3 | CO+hold | 8.00 | 7.08 |
| 1.0 m/s | 0 ms | Evader | 1 | LM+hold | 1.84 | 1.40 |
| 1.0 m/s | 0 ms | Evader | 1 | CO+hold | 1.22 | 1.20 |
| 1.0 m/s | 0 ms | Evader | 3 | LM+hold | 2.32 | 2.92 |
| 1.0 m/s | 0 ms | Evader | 3 | CO+hold | 2.44 | 2.46 |
| 1.0 m/s | 100 ms | Cycler | 1 | LM+hold | 2.26 | 1.68 |
| 1.0 m/s | 100 ms | Cycler | 1 | CO+hold | 3.10 | 2.40 |
| 1.0 m/s | 100 ms | Cycler | 3 | LM+hold | 4.70 | 7.26 |
| 1.0 m/s | 100 ms | Cycler | 3 | CO+hold | 4.24 | 6.62 |
| 1.0 m/s | 100 ms | Evader | 1 | LM+hold | 1.88 | 1.50 |
| 1.0 m/s | 100 ms | Evader | 1 | CO+hold | 1.82 | 1.22 |
| 1.0 m/s | 100 ms | Evader | 3 | LM+hold | 2.48 | 4.74 |
| 1.0 m/s | 100 ms | Evader | 3 | CO+hold | 2.36 | 1.78 |
| 4.5 m/s | 0 ms | Cycler | 1 | LM+hold | 2.82 | 1.96 |
| 4.5 m/s | 0 ms | Cycler | 1 | CO+hold | 2.42 | 1.66 |
| 4.5 m/s | 0 ms | Cycler | 3 | LM+hold | 2.76 | 2.30 |
| 4.5 m/s | 0 ms | Cycler | 3 | CO+hold | 3.74 | 4.34 |
| 4.5 m/s | 0 ms | Evader | 1 | LM+hold | 1.66 | 1.62 |
| 4.5 m/s | 0 ms | Evader | 1 | CO+hold | 0.52 | 0.50 |
| 4.5 m/s | 0 ms | Evader | 3 | LM+hold | 2.12 | 2.00 |
| 4.5 m/s | 0 ms | Evader | 3 | CO+hold | 2.64 | 2.32 |
| 4.5 m/s | 100 ms | Cycler | 1 | LM+hold | 3.00 | 2.56 |
| 4.5 m/s | 100 ms | Cycler | 1 | CO+hold | 2.44 | 2.10 |
| 4.5 m/s | 100 ms | Cycler | 3 | LM+hold | 4.36 | 2.96 |
| 4.5 m/s | 100 ms | Cycler | 3 | CO+hold | 3.14 | 4.28 |
| 4.5 m/s | 100 ms | Evader | 1 | LM+hold | 1.46 | 1.44 |
| 4.5 m/s | 100 ms | Evader | 1 | CO+hold | 0.38 | 0.38 |
| 4.5 m/s | 100 ms | Evader | 3 | LM+hold | 2.46 | 2.08 |
| 4.5 m/s | 100 ms | Evader | 3 | CO+hold | 2.46 | 2.46 |

Totals: episodes over 2.5 s **199 → 78**; over 3.0 s 62 → 43. One ghost: at or under 2.5 s in
22 of 24 setups after (over: 0.5 m/s Evader 2.98 s, 4.5 m/s + 100 ms Cycler 2.56 s), 19 of 24
before (worst 3.30 s). Three ghosts: still 2.3–14.4 s. Data:
[data/pin-margin-study-before.txt](data/pin-margin-study-before.txt),
[data/pin-margin-study-after.txt](data/pin-margin-study-after.txt), worst run after:
[data/pin-margin-worst-timeline-after.txt](data/pin-margin-worst-timeline-after.txt) (0.5 m/s +
100 ms, 3 Evaders, Line mirror + hold, 14.4 s).

## Tried and dropped (same grid, 48 runs each; episodes over 2.5 s, referee pins)

| Change | Over 2.5 s | Referee pins | Kept? |
| --- | --- | --- | --- |
| Old margin (main) | 199 | 0 | — |
| New margin | **78** | 0 | **yes** |
| New margin, back off 0.9 s earlier still (latency set to 1.0 s) | 97 | 0 | no: earlier is not better |
| + never back off into another robot | 79 | 1 | no |
| + also prefer the side away from the attacker's lane to the goal | 91 | 1 | no |
| + tell the guard when the geofence stops the back-off | 106 | 0 | no |
| + geofence as walls for the back-off | 253 | 54 | no |
| Margin from a predicted time-to-clear (16 directions, others moving) | 78 | 0 | no: same as the simple rule |
| Back-off direction from that prediction | 280 | 131 | no |

Small changes to the back-off move the 3-ghost results a lot in both directions. That is why
the margin is kept simple and the back-off redesign is a separate job. (Note for anyone running
experiments: Gradle's daemon keeps `-D` system properties between builds. Stop it
(`./gradlew --stop`) or pass every switch on every run.)

## Cost in the simulator

The sim robot now backs off at 1.91 s of contact instead of 2.5 s.

- Pins (`--batch-anon 180 5`, anonymous tracking + contact sensing, both setups, both cameras, 3
  fields, Cyclers + Evaders): **16 → 11 in 120 runs without partners, 22 → 10 with partners**.
  Known ids: 17 → 6 and 17 → 3.
- Slowed, same grid, anonymous camera: mean −0.9 points (no partners) and −0.3 (partners).
- Slowed with perfect tracking drops more: Line mirror + hold vs Cyclers at the choke +51% → +27%,
  vs Evaders in two lanes +70% → +45%; Cut-off + hold vs Cyclers in open field +46% → +36%. The
  full tables are in README "Current findings".
- New: Cut-off + hold, two lanes, Cyclers, Realistic camera gridlocked in 2 of 5 runs (the
  guarded opponent never finished a cycle in one).
- `SimTest.shadowStaysGoalSideAndNumbersStayFinite`: goal-side fraction 0.709 → 0.677, bar
  lowered 0.7 → 0.65.

## Next

Done in [backoff-redesign.md](backoff-redesign.md) (one ghost fixed, three not). Original note:
redesign the back-off for crowds: treat other robots as obstacles, don't retreat along a
pusher's path, know the geofence on the robot. Re-run `PinMarginStudy` (target: 3 ghosts at or
under 2.5 s) plus `--headless` and `--batch-anon` after it.
