# PROGRESS — branch `anon-tracking` (anonymous opponent tracking)

Working copy: the git worktree at `../pt-tracking` (branch `anon-tracking`), not the main checkout.
Task brief: remove the "we know which opponent every sighting belongs to" assumption
(`Measurement.id`). Stopped early at the owner's request; this file says exactly where.

Last verified state: `./gradlew test` = **138 pass** (115 old + 23 new). `./gradlew run
--args="--headless"` output is **byte-identical to main** (nothing in `phantom.sim` or the modes was
touched yet, so no defense number has changed).

## Done and tested (phantom.core.tracking, zero dependencies)

| Piece | File | Tests (hand-computed values in comments) |
| --- | --- | --- |
| Hungarian assignment, O(n^3), rectangular | `Hungarian.java` | `HungarianTest`: 3x3, 3x3 where greedy is wrong, 3x2, 2x3, negative + forbidden costs, bad input, 2000 random matrices vs brute force |
| Mahalanobis gate, 2 DOF, threshold 9.210 with source | `Gating.java` | `GatingTest`: 9.16 accepted / 9.36 rejected, per-axis variance, boundary, broken covariance -> reject not NaN |
| Non-mutating Kalman "peek" (`predictedPosition`, `predictedPositionVar`) | `AxisKalman.java` (additive) | `GatingTest.axisKalmanPeek...` |
| Alliance color as clamped log-odds | `ColorBelief.java` | `ColorAndConfirmationTest`: update sequence at e = 0.1, prior, clamp, flips needed, label likelihood, edge error rates |
| M-of-N confirmation (default 3 of 5) | `MOfN.java` | `ColorAndConfirmationTest`: six 3-of-5 sequences, sticky decision, other sizes |
| Anonymous detection types | `Detection`, `DetectionFrame`, `DetectionSource`, `AllianceLabel` | used by the tracker only |

## Written and compiling, but NOT tested and NOT used anywhere

`MultiTargetTracker.java` implements `OpponentTracker` (interface unchanged): GNN association each
frame (cost = d2 + ln det S - 2 ln P(label | track), gated at 9.210, solved with `Hungarian`),
tentative -> confirmed -> coasting -> deleted, stable ids, per-track color belief, reports only
tracks with P(opponent) > 0.5, confidence = P(opponent) * s0^2 / (s0^2 + var), frames applied at
capture time, out-of-order frames dropped, and re-acquisition (a new track takes over the id of a
recently lost one by proximity + velocity).

**Treat it as a draft.** It has no unit tests, has never run against simulated data, and is not
wired into `SimSession`. Its re-acquisition defaults (2 s window, 1.5 m radius, 0.5 s lead, no
velocity gate) are guesses. `HungarianTest` uses its `FORBIDDEN` constant, nothing else touches it.

Problems I found reading it back (not fixed):

- **Handoff often won't fire.** It only runs at the moment a track is first reported. If the
  replacement track is confirmed while the old track is still coasting (coasting can last up to
  1 s, confirming takes about 0.1 s), the old id is not carried over. That is probably the usual
  order of events. Fix idea: when a reported track times out, let a track born near it during its
  coast take over its id.
- **Pin timers are keyed by track id** (`RulesGuard` / `PinTimer`). An id change during contact
  restarts that opponent's pin timer, and only the stall-based blind back-off would still catch the
  pin. Pins must be measured before hold is used with anonymous tracking.

## Not started

1. **Tracker tests** (math rules: hand-computed, arithmetic in comments). Planned cases, numbers
   already checked numerically:
   - GNN vs greedy: R = 0.2, accel 0, tracks at x = 0 and 1, detections at 0.9 and 2.0 after
     0.1 s -> S = 0.44; d2 = 1.841 / 9.091 / 0.023 / 2.273; totals 4.114 vs 9.114; expect
     A.x = 0.4909, B.x = 1.5455.
   - Gate through the tracker: R = 0.01, dt = 0.1, accel 0 -> S = 0.06; 0.74 m -> 9.127 (same
     track), 0.75 m -> 9.375 (new track).
   - ln det S term: sharp track S = 0.06 (d2 = 6.0, cost 0.373) must beat coasting track
     S = 1.02 (d2 = 5.647, cost 5.687).
   - Color-aided tie: equidistant detection, d2 = 0.568, costs -0.677 (matching color) vs 2.356.
   - Confidence: 0.9 * 0.25 / 0.26 = 0.8654 at age 0, 0.9 * 0.25 / 1.51 = 0.1490 at 0.5 s.
   - Confirmation through frames, latency timestamp, out-of-order frame, timeout at exactly 1.0 s,
     partner never reported, handoff accept / too far / too late / disabled, NaN detections, fuzz.
2. **Sim realism** (`phantom.sim`): optional alliance partners (off by default), `AnonymousSensor`
   (no ids, color label with error rate, Poisson clutter, occlusion, own-robot removal),
   Realistic / Bad presets with documented clutter and color-error numbers, occlusion geometry test.
3. **Wiring**: `SimSession` option to use `AnonymousSensor` + `MultiTargetTracker`.
4. **Metrics** in `--batch` CSV and headless: position RMSE, ID switches/min, false-track time,
   per-opponent coverage, partner-tracked-as-opponent time.
5. **Comparison table** (5 seeds, 180 s, known-ID Kalman vs MultiTargetTracker) and
   `docs/findings/anonymous-tracking.md`, including the list of what still differs from a real
   camera.
6. **Measuring re-acquisition** and choosing its defaults.
7. "No NaN reaches a command" test with the new tracker in the loop.

## Open questions (decisions I was about to make; none is implemented in the sim yet)

- **What does "locked on #0" mean without ids?** Known-ID tracking re-identifies #0 for free every
  time it comes back into view. Anonymous tracks cannot. Plan was: the sim plays a driver who
  re-designates the target after a reaction delay, and the report counts re-designations/min.
  Needs the owner's view on how a real driver would pick a target.
- **Partners "toward our goal"**: read as our alliance's own goal = the defended goal mirrored
  through field center, partners picking up at the mirrored pickups. Not confirmed.
- **Partner driving style**: planned Evader (dodges), because a Cycler partner would ram our own
  defender. Not confirmed.
- **Clutter / color-error presets**: planned 0.05 false detections per frame + 5% color error
  (Realistic), 0.20 + 15% (Bad). Pure guesses, no measurement behind them.
- **Truth matching for metrics**: planned CLEAR-MOT style matching with a 1.0 m radius.
- **Merge note**: the other two parallel branches may also add a root `PROGRESS.md`; expect an
  add/add conflict there. This file can be deleted at merge.

## Scope rules being followed

README "Current findings" and CLAUDE.md untouched. `DefenseMode`, the modes, `HolonomicController`
and the `OpponentTracker` interface untouched. Only existing file changed: `AxisKalman.java` (two
added read-only methods).
