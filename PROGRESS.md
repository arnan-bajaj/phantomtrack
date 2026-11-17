# contact-sensing branch: progress

Goal: close the pin-guard safety hole (pins slip through with a noisy camera near walls, and the
back-off can point into a wall). One of three parallel branches; this one must not touch
README "Current findings" or CLAUDE.md. Results go to `docs/findings/contact-sensing.md`.

## Milestones

- [x] 0. Study harness (`phantom.sim.ContactStudy`) and baseline numbers on unchanged code
- [ ] 1. Sim drive telemetry: commanded/measured velocity, acceleration, DC-motor current, noise
- [ ] 2. `ContactDetector` (core, pure): stall + current + deceleration + vision, with hysteresis
- [ ] 3. `PinTimer` / `RulesGuard` use the detector (replaces camera-only and stall-only contact)
- [ ] 4. Wall-aware back-off (left / right / straight back, 0.5 s look-ahead)
- [ ] 5. Results over 10 seeds, findings doc, calibration procedure
- [ ] 6. PR (not merged)

## Baseline (commit of milestone 0, unchanged algorithms)

`java -cp build/classes/java/main phantom.sim.ContactStudy 180 10` after `./gradlew classes`:
338 pins in 240 runs (1.41 per 180 s run). Worst: Choke, Cyclers, Cut-off + hold, Bad camera,
4.9 pins per run. Full table: `docs/findings/data/contact-study-before.txt`.

## Notes

- Where baseline pins come from (scratch diagnostic, 338 pins): about two thirds are on the
  guarded opponent while it IS tracked, because camera contact (estimated gap <= 0.05 m) flickers
  and each flicker loses time; about one third are on a different opponent that is beside or
  behind us and often not tracked at all, sometimes because the back-off drove into it.
