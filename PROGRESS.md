# PROGRESS — branch `anon-tracking` (anonymous opponent tracking)

Working copy: the git worktree at `../pt-tracking` (branch `anon-tracking`). Draft PR #11.
Goal: drop the assumption that every sighting comes with the right opponent id
(`Measurement.id`). Results and the list of what still differs from a real camera are in
[`docs/findings/anonymous-tracking.md`](docs/findings/anonymous-tracking.md).

Last verified: `./gradlew test` = **174 pass** (115 old + 59 new). `./gradlew run
--args="--headless"`: all old sections byte-identical to main; three new anonymous-tracking
sections at the end.

## Done

| Brief item | Where | Tests |
| --- | --- | --- |
| 1. Partners (off by default) | `SimWorld` partners, `Scenarios.three(..., withPartners)` | `AnonymousSensorTest` (layout, cycling) |
| 1. AnonymousSensor: no ids, color label + error rate, Poisson clutter in the FOV, occlusion, own-robot removal; Realistic / Bad presets | `AnonymousSensor` | `AnonymousSensorTest`: occlusion geometry by hand (straight + diagonal), hidden robot, labels/flips, own robot, Poisson mean, field filter, latency, seeding |
| 2. Hungarian, O(n^3), rectangular | `core/tracking/Hungarian` | `HungarianTest`: 3x3, 3x3 vs greedy, 3x2, 2x3, brute-force check |
| 2. Mahalanobis gate, 2 DOF, 9.210 with source | `Gating` | `GatingTest`: 9.16 accepted / 9.36 rejected |
| 2. MultiTargetTracker: GNN, tentative/confirmed (3 of 5)/coasting/deleted, stable ids, log-odds color, report opponents only, confidence, latency/out-of-order | `MultiTargetTracker`, `ColorBelief`, `MOfN` | `MultiTargetTrackerTest`, `ColorAndConfirmationTest` (log-odds sequence, M-of-N sequences, same numbers as KalmanOpponentTracker for one robot) |
| 3. Re-acquisition (proximity + elimination), measured | `MultiTargetTracker` handOff / eliminate; `--tracker-sweep` | handoff gate by hand, accept / too far / too late / off, coasting merge, color flip, elimination |
| 4. Metrics in headless + batch CSV: RMSE, ID switches/min, false-track time, coverage per opponent, partner-as-opponent time | `SimSession.Metrics`, `matchTracks` (CLEAR-MOT), `BatchRunner` | `AnonymousTrackingSimTest`: scorer by hand, metrics by hand on a scripted tracker |
| 5. Comparison table (5 seeds, 180 s, both cameras, partners, Cyclers/Evaders, open/choke, Line mirror auto + Cut-off) | `SimMain.runAnonymousComparison` (headless), findings doc | — |
| No NaN to a command | — | `noNaNReachesACommandWithTheAnonymousTrackerInTheLoop` (all modes, Bad, choke, partners) |
| Findings + honest list of differences | `docs/findings/anonymous-tracking.md` | — |

Extras beyond the brief:
- "Known-ID + occlusion" tracking to separate occlusion from anonymity.
- A simulated driver for "locked on #0" without ids, with re-points per minute.
- Camera-coverage and coverage-breakdown sweeps.
- Window keys T (tracking) and B (partners).

## Next (if someone continues)

1. **Pin timers are keyed by track id** (`RulesGuard` / `PinTimer`). Pins were not worse with
   anonymous tracking in the sim, but an id change during contact does restart that opponent's
   timer. Worth a unit test and maybe a by-position fallback.
2. **Persistent false positives** (the same spot every frame) are the realistic worst case and are
   not simulated. Add a "fixed clutter points" option and re-run the comparison.
3. **Multiple real cameras** need fusing before this tracker (one robot seen by two cameras gives
   two detections in one frame).
4. Measure the real pipeline (detection rate vs distance, color error, false positives per
   frame, latency) and replace the guessed preset numbers.

## Open questions for Arnan

- Is the simulated driver for man-to-man (re-points 0.5 s after the assist loses the robot or
  locks onto the wrong one, and always knows which robot is which) a fair stand-in for a human
  with a "guard this robot" button?
- Partner layout: mirrored, loading at the far end and crossing our zone, driving like Evaders.
  Does that match what you expect from 2027?
- Re-acquisition by elimination assumes exactly 3 opponents. If a robot is pulled off the field
  mid-match, ids could be given to the wrong robot more often. Keep it on by default?
- Merge note: the other two parallel branches may also add a root `PROGRESS.md` (add/add
  conflict). This file can be deleted at merge.

## Scope rules followed

README "Current findings" and CLAUDE.md untouched. Other README sections got a few lines (new
commands, keys, classes, a roadmap line). `DefenseMode`, the modes, `HolonomicController` and
the `OpponentTracker` interface untouched. `AxisKalman` got two read-only methods.
