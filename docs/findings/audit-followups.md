# Audit follow-ups (Nov 18): M5, U1, the robot tracking test

Branch `audit-followups`, after PR #17. The audit is in [../audit-2026-11.md](../audit-2026-11.md).

**Result: U1 is kept.** Over 10 paired seeds, slowed % moved by −0.1 ± 0.2 points (the seed
spread is 0.5), so it is not worse beyond the spread. Total contact fell by 0.5 ± 0.15 s per run.
Peak contact and pins did not change. M5 is fixed. Looking at the one 22.5 s outlier in the first
U1 comparison showed that the first M5 fix missed jams with three robots, so that check was fixed
too.

## M5: two Cyclers head-on in a gap (sim only)

`CyclerBehavior` (formula in its class comment, tests in `HeadOnGapTest`):

- **When it fires:** two Cyclers touch inside a gap (bumper gap at most 0.05 m, and their midpoint
  within 1.0 m of a gate). Both drive into each other (effort above 0.5). Neither moves 0.3 m or
  more, net, in 1.0 s.
- **Who backs out:** the lower id, to `Y = clampInside(gate − 1.5 u + 1.2 side)`.
  - `u` is the direction to the gate.
  - `side` points away from the other robot's destination.
- **How long:** until the other has passed (`(p_other − p_me)·u < 0`), or at most 5 s.
- **What it never does:** a Cycler never backs out from our robot or from a partner. Evaders are
  unchanged.

Results:
- **The audit's failing input** (two lanes, no defense, 10 ms) now does about 22–25 cycles per
  opponent. Before, #0 and #2 were stuck from t ≈ 27 s with 3 and 4 cycles.
- **Choke with partners**, Line mirror + hold locked #0, Cyclers (realistic camera, anonymous):
  +55% ± 37 → +22% ± 2. The +55% came from a jammed baseline-like run.
- **Two lanes with partners**, Cyclers: slowed went from about −27% to −19%. With partners in
  the lanes, "slowed" is still negative and still not meaningful there (README).

**Second fix, found through U1.** The first version tested "stalled" with the body's velocity
(under 0.3 m/s). In one `--batch-anon` run (Cyclers, two lanes, Cut-off + hold, partners,
Realistic, anonymous, seed 5):
- our robot was wedged at the bottom of the lower gap, yielding;
- two Cyclers were head-on above it;
- collisions kept their velocities at 0.3–1.0 m/s while their positions moved by 0.01 m a step.

The rule never fired, and the referee counted **22.5 s** of contact in one episode (146 s of
total contact). The check now uses net displacement over the 1 s window. That run drops to a 1.9 s
peak and 0 pins. A hand-built copy of the jam is in `HeadOnGapTest`: 0 cycles in 30 s before the
fix, 4 each after. The U1 comparison below was re-run with this fix in both "before" and "after".

## U1: hold ground pushes only on opponents closing on us or on the goal

```
closing  = max(-v_opp . unit(p_opp - p_self), v_opp . unit(goal - p_opp))
released when closing < -0.25 m/s on every update for 0.2 s
allowed again as soon as closing > -0.10 m/s
```

- **Why −0.25 m/s, not 0:** in a stalled pushing match the closing speed is 0, and hold has to
  keep pushing there.
- **Why 0.2 s, and the gap up to −0.10 m/s:** camera velocity noise around −0.25 m/s cannot switch
  the hold back and forth.
- **Where the state lives:** `HoldLatch` in the manager, so `HoldGroundMode` stays a pure function.
  It reads the released ids from `DefenseContext.holdReleased`.
- **Tests:** `HoldLatchTest`, with hand-computed values. A velocity alternating −0.30 / −0.20 m/s
  for 100 loops gives 0 switches with the hysteresis, against 100 with an instant rule.

### How it was compared

- **Before:** commit `9e14603` (M5 with both fixes, plus the new measurements). **After:**
  `69d5f44` (U1). Same seeds, compared pair by pair.
- **`--batch-anon 180 10`:** 1440 camera runs paired by (opponents, setup, field, partners,
  camera, tracking, seed), plus 36 perfect-tracking runs.
  - **Pooled:** for each seed, the mean over the 144 camera setups; then the mean ± standard error
    of the 10 per-seed differences.
  - **Per setup:** a difference is flagged when it is more than 2 SE.
- **`PinMarginStudy`, one ghost:** 24 setups × 300 s × seeds 0–9.
  - Seed 0 is the fixed start used until now.
  - Other seeds move our start by up to ±0.5 m and engage up to 3 s later.
- Raw data and the comparison scripts are in [data/](data/), files `u1-*`.

### `--batch-anon` (camera rows, 10 seeds, 1440 paired runs)

| Metric (mean per run) | Before | After | Paired difference ± SE | Seed spread (sd of the before mean) |
| --- | --- | --- | --- | --- |
| Slowed % | +11.47 | +11.35 | **−0.12 ± 0.18** | 0.49 |
| Total contact | 34.63 s | 34.12 s | **−0.51 ± 0.15 s** | 0.33 s |
| Peak referee contact (longest episode) | 2.08 s | 2.05 s | −0.03 ± 0.02 s | 0.05 s |
| Referee pins | 0.060 | 0.060 | +0.000 ± 0.006 | 0.014 |
| Hold switches on/off per min | 30.7 | 29.8 | **−0.92 ± 0.11** | 0.40 |

- **Pins:** 87 → 87 in total. The README headline (anonymous; Line mirror + hold auto and
  Cut-off + hold), 240 runs each:
  - without partners: 18 → 19;
  - with partners: 20 → 27. Per-seed differences there are −4 to +2, mean +0.7 ± 0.6 per seed.
- **Episodes over 3 s:** 82 → 81 runs. Longest: 6.36 s → 4.48 s.
- **Per setup:** 6 of 144 setups are worse by more than 2 SE and 5 are better. At 2 SE with 10
  seeds, that is about what chance gives on both sides.
- **The hold permission itself (`HoldLatch`)** flips 42 times a minute on average. It tracks every
  opponent, including far-away ones, so each opponent turning around after scoring or loading is
  two flips. It is not flicker: the push itself switches less often (table).

### Perfect tracking (deterministic, single runs, 36 setups)

- **Slowed:** +23.2% → +25.1% on average, but the per-setup differences have a standard deviation
  of 10.7 points (largest: Evaders, two lanes, Line mirror + hold locked #0, +54% → +88%). These
  runs are chaotic, so they are reported here and not used for the decision.
- **Contact:** 45.9 → 45.5 s. **Peak referee contact:** 1.90 → 1.88 s. **Pins:** 0 → 0.
- **Hold switches:** 23.7 → 27.4 a minute. With exact velocities there is no noise. The extra
  switches are real releases: an opponent drives away, then comes back.

### `PinMarginStudy`, one ghost (24 setups × 10 seeds × 300 s)

| Metric (mean per run) | Before | After | Paired difference ± SE |
| --- | --- | --- | --- |
| Slowed % | +11.2 | +11.2 | 0 (identical in all 240 pairs) |
| Total contact (any touch) | 28.42 s | 28.44 s | +0.02 ± 0.01 s |
| Peak contact (any touch, longest episode) | 1.33 s | 1.34 s | +0.004 ± 0.003 s |
| Peak referee contact | 0.91 s | 0.89 s | −0.03 ± 0.003 s |
| Referee pins | 0 | 0 | 0 |
| Hold switches on/off per min | 9.26 | 10.13 | +0.86 ± 0.01 |
| Back-offs | 21.3 | 20.7 | −0.6 |

Slowed counts whole ghost cycles (one cycle is 3–17 points), and no seed moved it. The ghosts'
velocities are exact here as well, so the extra hold switches are real releases, as in the
perfect-tracking rows.

**Decision:** slowed % is not worse beyond the seed spread in either study, so U1 is kept.
- **What it gains:** about 0.5 s less contact per 180 s run with a camera, and slightly less
  referee time per episode.
- **What it does not change:** pins.
- **Hold switches:** about 1 a minute fewer with a camera; with exact velocities, 1–4 a minute
  more.

### Found while seeding `PinMarginStudy` (not caused by this branch)

PR #16 found every one-ghost setup at or under 2.5 s of any-touch contact. That held only for the
one fixed start:
- **With seeds:** 12 of 240 runs go over 2.5 s, in 3 of 24 setups. The maximum is 3.52 s (Cut-off
  + hold, 1.0 m/s, 100 ms). Then 3.04 s and 3.00 s (Line mirror + hold, 4.5 m/s).
- **Same before and after U1.**
- **Referee:** the longest referee time stays at or under 1.84 s, and there are no referee pins.

## Robot closed-loop tracking test (`PhantomTrackTest`)

Restored to its original scope: every loop after the first 10 s, 2500 loops, Line mirror with no
hold against one 1 m/s Cycler. The test splits those loops into two numbers, each with its own
bar, and fails if any loop is in neither:

| | Measured | Bar |
| --- | --- | --- |
| Tracking error while the mode is in charge (1864 loops, including the recovery after back-offs) | 0.358 m | < 0.38 m |
| Fraction of the time backing off (636 of 2500 loops = 12.7 s of 50 s) | 0.254 | < 0.27 |

The old bar was 0.25 m, and it held only because #13 dropped the 3 s after each back-off. That
was 37% of the samples, and the back-off time itself was not scored at all. U1 does not affect
this test: there is no hold in it.
