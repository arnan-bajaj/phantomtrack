# PhantomTrack — rules for AI-assisted development

This project is mostly written with AI help by a student who reviews but does not hand-write most
code. Keep changes small, tested and explained.

## Architecture rules (do not break)

1. `phantom.core` has **zero dependencies**: no WPILib, no AWT/Swing, no imports from
   `phantom.sim`. It must compile and run inside a WPILib robot project unchanged.
2. Dependency direction is one-way: `sim` may import `core`; `core` never imports `sim`.
3. Defense modes (`DefenseMode`) are **pure functions** of `DefenseContext`: no static state, no
   clock reads, no randomness, no hardware.
4. Defense code sees opponents only through `OpponentTracker`. Never read `SimWorld` truth from
   `core`.
5. Field geometry lives only in `FieldConfig`. No hardcoded field coordinates elsewhere in `core`.
6. Coordinates follow WPILib: meters, origin at blue alliance wall corner, +x toward red, +y left,
   angles CCW radians wrapped to (-pi, pi] via `Angles.wrap`.
7. Java 17 language level (WPILib's JDK). No new main dependencies.

## Math rules (zero tolerance for arithmetic errors)

- Every new geometry or control formula gets a unit test with **hand-computed expected values**,
  and the arithmetic is written in a comment next to the assertion
  (e.g. `// u = (6,0)/6 = (1,0), target = (2,4) + 1*(1,0) = (3,4)`).
- Test the edge cases: zero-length vectors, opponent on the goal, opponent faster than us, walls.
- Never let NaN reach a command: use `Vec2.normalized()` (returns ZERO, not NaN) and check
  `isFinite()` in tests.
- Write the formula in the class Javadoc so the code can be checked against it.

## Experiments and tests

- **Never loosen an existing test threshold.** If a test fails after a change, stop and report
  why (which test, what it expected, what it got, and what in the change caused it) instead of
  changing the bar.
- **No `-D` system properties for experiments.** Gradle's daemon keeps them between builds, so a
  switch can silently stay on. Pass switches as program arguments (e.g.
  `./gradlew pinMarginStudy --args="..."`, `./gradlew run --args="..."`), or run with
  `--no-daemon`.

## Before every commit

1. `./gradlew test` passes.
2. `./gradlew run --args="--headless"` runs, and the metrics table is reported in the summary of the
   change (did the change help or hurt?).
3. If a change alters behavior, update the "Current findings" table in README.md.

## Roadmap status

See README.md. Done: Phase 1 (sim), Phase 4 (modes: Shadow, Interpose, Zone, Line mirror, Cut-off),
Evader opponent and pushing-match contact model, hold ground + RulesGuard pin timer, Phase 2
(simulated camera + Kalman filter, search behavior), TargetSelector (auto focus), `--batch` CSV
export, human reaction delay for Evaders, walls/chokepoints/lanes with gate routing,
IntentPredictor + Cut-off mode. Nov 16 work, integrated: contact sensing (ContactDetector fuses
drivetrain stall/current/deceleration with the camera; wall-aware EscapePlanner back-off),
anonymous tracking (AnonymousSensor with partners, color errors, clutter, occlusion;
MultiTargetTracker with re-acquisition), pin time keyed by contact episode (EpisodeKeys) so it
survives track id changes, a pin-guard margin from the robot's back-off time and command latency
(`RulesGuard.safetyMarginFor`: 0.5 s slack + latency + loop + time to slide one robot width; same
rule in sim and robot, `robot/.../PinMarginStudy` measures it), and Phase 7 software (`robot/`: WPILib project, ghost opponents
against the real pose, safety rules; it compiles phantom/core plus 8 AWT-free sim files listed in
robot/shared-sources.txt, checked by RobotSharedSourcesTest). Nov 17: back-off for crowds
(EscapePlanner sees every robot and where it will be, never along a pusher's path, 16 extra
directions, yield when nothing is safe then slide out after 1 s; `PinMarginStudy` runs as
`./gradlew pinMarginStudy --args=...`). Nov 18: audit (docs/audit-2026-11.md): NaN tracks dropped, red
driver's sticks rotated on mirrored fields, Shadow/Interpose no longer lead an opponent away
from the goal, rotation overrides the blend, clampInside and hold targets legal near walls,
seeded randomized invariant tests (core InvariantTest, robot MirrorAndDriverInvariantTest).

Key findings: Line mirror + hold is the safe default and is best or tied everywhere with a real
(anonymous) camera; Cut-off + hold no longer wins anywhere with a camera; a realistic camera costs
about half the effect and anonymity mostly costs man-to-man (driver re-points ~7.4/min). Open
problems: pins are not zero (0.03-0.09 per 3 min with anonymous tracking + contact sensing, from
camera error near contact); every contact threshold, camera number and the command latency are
placeholders; with 3 ghosts the back-off still lets contact reach ~5 s against Cyclers (and 11 s
in one Evader run stuck at the geofence, which the core does not know about), while 1 ghost stays
under 2.5 s everywhere and the referee count is 0 (docs/findings/backoff-redesign.md).
Remaining items need the real robot (Phase 7 test session, contact calibration, latency and camera
measurements, Phase 8 vision) or the 2027 game (FieldConfig, PinRules).

## Workflow

- Work on a branch, open a PR with the metrics table in its description.
- The repo owner has given permission for Claude to merge its own PRs once `./gradlew test` passes
  and the headless run has been reported.
