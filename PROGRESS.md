# PROGRESS — branch `anon-tracking` (anonymous opponent tracking)

Working copy: the git worktree at `../pt-tracking` (branch `anon-tracking`), not the main checkout.
Task brief: remove the "we know which opponent every sighting belongs to" assumption
(`Measurement.id`). Work continues on this branch; this file says where it stands.

Last verified state: `./gradlew test` = **155 pass** (115 old + 40 new). `./gradlew run
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

## MultiTargetTracker: tested, not wired into the sim yet

`MultiTargetTracker.java` implements `OpponentTracker` (interface unchanged): GNN association each
frame (cost = d2 + ln det S - 2 ln P(label | track), gated at 9.210, solved with `Hungarian`),
tentative -> confirmed -> coasting -> deleted, stable ids, per-track color belief, reports only
tracks with P(opponent) > 0.5, confidence = P(opponent) * s0^2 / (s0^2 + var), frames applied at
capture time, out-of-order frames dropped.

Re-acquisition: a newly reported track takes over the id of a lost track (ghost, kept 2 s) OR of
a reported track that is still coasting, if it is within 1.5 m of where that track should be and
its velocity is within 3 m/s. The coasting copy is then deleted. (The coasting case fixes the
"handoff often won't fire" problem from the first commit.) The numbers are not measured yet.

`MultiTargetTrackerTest` (17 tests, hand-computed): GNN beats greedy, gate in/out through the
tracker, ln det S stops a stale track stealing, color label breaks a tie, 3-of-5 / coast /
re-confirm, tentative rejection, partner never reported, timeout, latency + out-of-order,
**identical to KalmanOpponentTracker for one robot**, confidence, handoff gate by hand, handoff
accept / too far / too late / off, coasting merge, color flip releases and reclaims the id, NaN
input, 3000-frame garbage fuzz.

Still open: **pin timers are keyed by track id** (`RulesGuard` / `PinTimer`). An id change during
contact restarts that opponent's pin timer; only the stall-based blind back-off would still catch
the pin. Pins must be measured before hold is used with anonymous tracking.

## Not started

1. **Sim realism** (`phantom.sim`): optional alliance partners (off by default), `AnonymousSensor`
   (no ids, color label with error rate, Poisson clutter, occlusion, own-robot removal),
   Realistic / Bad presets with documented clutter and color-error numbers, occlusion geometry test.
2. **Wiring**: `SimSession` option to use `AnonymousSensor` + `MultiTargetTracker`.
3. **Metrics** in `--batch` CSV and headless: position RMSE, ID switches/min, false-track time,
   per-opponent coverage, partner-tracked-as-opponent time.
4. **Comparison table** (5 seeds, 180 s, known-ID Kalman vs MultiTargetTracker) and
   `docs/findings/anonymous-tracking.md`, including the list of what still differs from a real
   camera.
5. **Measuring re-acquisition** and choosing its defaults.
6. "No NaN reaches a command" test with the new tracker in the loop.

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
