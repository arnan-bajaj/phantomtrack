# contact-sensing branch: progress

Goal: close the pin-guard safety hole (pins slip through with a noisy camera near walls, and the
back-off can point into a wall). One of three parallel branches. This branch does not touch
README "Current findings" or CLAUDE.md; results are in `docs/findings/contact-sensing.md`.

## Done

- [x] 0. Study harness (`phantom.sim.ContactStudy`) and baseline numbers on unchanged code
- [x] 1. Sim drive telemetry: commanded/measured velocity, acceleration, DC-motor current
      (`DriveMotorModel`, Kraken X60 default, NEO option), noise, 20 ms samples
- [x] 2. `ContactDetector` (core, pure): stall + current + deceleration + vision, with hysteresis
- [x] 3. `PinTimer` / `RulesGuard` use the detector (replaces camera-only and stall-only contact)
- [x] 4. Wall-aware back-off (`EscapePlanner`: left / right / straight back, 0.5 s look-ahead)
- [x] 4b. Unit tests with hand-computed values (ContactDetector, EscapePlanner, PinTimer contact
      API, guard handover, motor model)
- [x] 5. Results over 10 seeds (before / after / oracle), FP and missed time, findings doc,
      calibration procedure
- [x] 6. PR opened (not merged)

Result: 338 → 29 pins in 240 noisy-camera runs (target 0, not reached). Slowed within 3 points in
17/24 rows; the other 7 are explained in the findings doc (old numbers came partly from pins; an
exact-contact oracle shows the legal ceiling).

Baseline pins came (scratch diagnostic) about two thirds from the guarded opponent while it was
tracked (camera contact flickered and each flicker lost time), and one third from another opponent
beside or behind us, often untracked, sometimes because the back-off drove into it.

## Next (if this direction is accepted)

1. Close the last 29 pins. The breakdown is in the findings doc; the largest groups are missed
   contact (10) and episodes closed too early (7). Candidates: require two fresh estimates
   before closing an episode; keep pin time running while the drivetrain still feels resistance.
2. Choke/Cyclers/Cut-off/Realistic is 4.7 points under the legal ceiling: cut false-positive
   back-offs there.
3. Perfect-tracking loss vs dodgers at the choke (Cut-off + hold, about 13 points over 360 s).
4. Update README "Current findings" after merge (deliberately not done on this branch).
5. On the robot: run the calibration procedure, replay the logs through `ContactDetector`.

## Open questions for Arnan

- Is the `SimTest` goal-side bar change (0.8 → 0.7) acceptable? On that 30 s run Shadow alone
  reads 0.749, so the old bar only passed thanks to a lucky back-off.
- Is "slide out past an opponent after 1 s wedged" acceptable rules-wise, or should the robot
  just stand still (legal, but it gridlocked a lane in the sim)?
- Should the oracle (guard sees truth) become a proper sim option, so the "legal ceiling" can be
  reproduced from the repo? It is a 3-line change in `DefenseAssist` and `SimSession`.
- `DefenseAssist` was changed (passes telemetry, detector config), which is outside this branch's
  planned file list. OK?
