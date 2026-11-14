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

## Before every commit

1. `./gradlew test` passes.
2. `./gradlew run --args="--headless"` runs, and the metrics table is reported in the summary of the
   change (did the change help or hurt?).
3. If a change alters behavior, update the "Current findings" table in README.md.

## Roadmap status

See README.md. Done: Phase 1 (sim), Phase 4 (four modes: Shadow, Interpose, Zone, Line mirror),
Evader opponent and pushing-match contact model, hold ground + RulesGuard pin timer, Phase 2
(simulated camera + Kalman filter, search behavior, stall detection). Key findings: holding ground
is what slows opponents; a realistic camera halves the effect; the pin guard needs contact sensing
beyond vision. Also done: TargetSelector (auto focus), `--batch` CSV export, human reaction delay
for Evaders, walls/chokepoints/lanes with gate routing (Line mirror's lead holds with walls),
IntentPredictor + Cut-off mode (best at chokepoints; prediction helps with a noisy camera). Open
problem: with a realistic camera near walls, 1-5 pins per 3 min slip through; needs real contact
sensing. Remaining roadmap
items need the real robot (Phase 7: AprilTag pose + ghost opponents) or the 2027 game (FieldConfig,
PinRules).

## Workflow

- Work on a branch, open a PR with the metrics table in its description.
- The repo owner has given permission for Claude to merge its own PRs once `./gradlew test` passes
  and the headless run has been reported.
