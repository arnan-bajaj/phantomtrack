# Phase 7: first robot session with ghost opponents

Goal of the day: show that PhantomTrack drives the real robot the way it drives in simulation,
and that every safety rule works on real hardware. **This is not a defense test.** Ghosts are
simulated, so nothing about defense quality is learned here.

Order: on blocks → carpet at 0.5 m/s → carpet at 1.0 m/s. Do not skip ahead. If any step fails,
stop, write down what happened, and fix it before continuing.

## Before you come

- [ ] `robot/` builds and `./gradlew test` passes; the drivetrain adapter is wired
      (`robot/README.md`, "Wiring a real drivetrain") and deployed. The Driver Station shows no
      "no real drivetrain wired" warning.
- [ ] The team's normal drive code was checked on this robot the same day (modules zeroed,
      gyro working).
- [ ] Laptop with the FRC Driver Station, AdvantageScope with `robot/advantagescope-layout.json`
      imported, and Elastic or Glass for the `PhantomTrack/Settings` values.
- [ ] A USB stick in the roboRIO (DataLogManager logs there if present), full batteries, bumpers
      on, an Xbox controller in DS slot 0.
- [ ] Tape, tape measure, a phone for video.

## People and where they stand

| Role | Where | Job |
| --- | --- | --- |
| **Safety officer** | At the DS laptop, outside the area, hands over the keyboard | **Enter = disable, Space = emergency stop.** Disables on any "STOP" call, any surprise, or anyone entering the area. Owns the "enable" decision. |
| **Driver** | Outside the outer tape, on the side the robot faces at the start mark, ≥ 2 m back | Holds the trigger. Lets go (or moves a stick) the moment anything looks wrong. |
| **Spotter / logger** | Outside the outer tape, at one side, ≥ 2 m back | Watches the robot, not the screen. Calls "STOP". Writes the trial log and films. |
| **Programmer** (optional) | Next to the safety officer | Watches AdvantageScope, reads out `Status`, changes settings only when told. |

Nobody is inside the outer tape while the robot is enabled. Anyone may call "STOP".

## The test area on carpet

PhantomTrack thinks in field coordinates. Pressing **Start** tells the pose estimator "the robot is
on the start mark": field (10.0, 4.035), facing −x (blue). The default test area is the 5 x 5 m
box around that mark; the assist will not drive the robot center past the **fence line** 0.75 m
inside it.

```
            outer tape: 5.0 x 5.0 m (test area)       driver stands on THIS side,
          +-----------------------------------+       in front of the robot
          |  inner tape: 3.5 x 3.5 m (fence)  |
          |   +---------------------------+   |
          |   |                           |   |
 field -x |   |        start mark  <==    |   | field +x
 (driver  |   |        (robot faces -x)   |   |
  side)   |   |                           |   |
          |   +---------------------------+   |
          +-----------------------------------+
```

- Mark the start mark and an arrow for the robot's facing. Tape the outer square (2.5 m from the
  mark each way) and the inner square (1.75 m each way). Leave at least 1.5 m of empty carpet
  outside the outer tape.
- On the mark (blue), field +x points out of the robot's **back** and field +y to the robot's
  **right**. Stick forward drives toward field +x, so the driver stands in front of the robot.
- Carpet too small? Shrink the area on the dashboard (`Area min/max X/Y`, still around (10, 4.035))
  and move the tape to match.
- With no AprilTags around the carpet, the pose is wheel odometry only. Turn vision updates off,
  or a bad tag sighting will look like a pose jump (the assist will switch off, which is safe but
  confusing).

## Rules for every trial

1. Before a trial: robot on the mark, trigger released, press **Start**, and check AdvantageScope
   shows the robot on the mark and `PoseHealthy` true.
2. Settings changes only with the trigger released (they wait otherwise; `SettingsPending`).
3. Assist speed cap and ghost settings are written in the log before every trial.
4. **Abort the session** (disable, power off, investigate) if: the robot moves with the trigger
   released and sticks centered; moves in a direction nobody expected; drives past the inner tape
   by more than a bumper width under the assist; `RobotPose` in AdvantageScope disagrees with
   where the robot really is by more than ~0.3 m; or the DS shows repeated loop overruns.

## Step 1: on blocks (wheels off the ground)

Robot on blocks, wheels free. Odometry will count the spinning wheels as motion, so the pose
moves in AdvantageScope as if the robot were driving. That is fine and lets the assist close its
loop. Press Start before each check. Settings: defaults (cap 1.0, 1 Cycler, Open, Line mirror).

| # | Do | Pass if | Fail = stop |
| --- | --- | --- | --- |
| 1.1 Wheel directions | Teleop, blue. Trigger released. Small stick pushes, one at a time: forward, left, turn left. | The pose in AdvantageScope moves toward field +x (away from the driver), field +y (the driver's left), and turns counter-clockwise. The wheels point and spin to match (on the mark, field +x is out the robot's back). | Any direction wrong: fix the drivetrain adapter (CTRE: forward perspective must be BlueAlliance). |
| 1.2 Deadman | Hold the trigger 10 s, release. | Wheels move only while held; `Status` = `ASSIST: ...` only while held; wheels stop at once on release. | Any wheel motion after release. |
| 1.3 Override | Hold trigger; while wheels are moving, push the left stick a little. Release the stick. | `DRIVER_OVERRIDE` and wheels follow the stick; on release `ASSIST` returns. | Wheels ignore the stick. |
| 1.4 Autonomous | DS Autonomous, enable, hold trigger 5 s. | Nothing moves; `Status` = `NOT_TELEOP`. | Any motion. |
| 1.5 Mirror on red | DS station Red 1. Trigger released, Start. Then hold the trigger. | `RobotPose` = (6.54, 4.04, 0°), `BlueFrame/RobotPose` = (10.00, 4.04, 180°), ghost and fence drawn on the red half; the robot (in AdvantageScope) drives toward the target arrow just as on blue. Pressing Start or switching alliance shows `POSE_FAULT` until the trigger is released and pressed again (expected). | Robot drives away from the arrow, or blue-frame pose is not (10.00, 4.04, 180°). |
| 1.6 Rumble | Back on blue. Line mirror, hold trigger, wait for the ghost to reach the robot. | Controller rumbles once `MaxPinSeconds` passes 1.5 s, `BACKING_OFF` by 2.5 s, rumble stops after. | No rumble, or rumble that never stops. |
| 1.7 Loop time | During 1.2–1.6 | No repeated loop-overrun warnings in the DS console. | Repeated overruns: try 1 ghost; report. |

## Step 2: carpet, 0.5 m/s

Dashboard: `Assist speed cap` = **0.5**, ghost count 1, Cycler, Open, ghost max speed 1.0.
Driver max speed stays 1.5 m/s (the driver is not capped; drive gently).

| # | Do | Pass if |
| --- | --- | --- |
| 2.1 Driver only | Trigger released, drive a small square inside the inner tape. | Stick forward = away from the driver; `RobotPose` follows the real robot within ~0.2 m. |
| 2.2 Track | Line mirror, hold trigger 60 s (3 trials). | Robot slides sideways along one line (field x ≈ 10, i.e. through the mark), follows the ghost's y, stays inside the inner tape. `CommandedSpeed` never above 0.5. `TargetDistance` small (< 0.3 m) whenever the target is inside the fence. |
| 2.3 Let go | While the robot is moving, release the trigger. | Stops within ~0.2 m. |
| 2.4 Take over | While moving, push the stick. | Robot follows the stick at once. |
| 2.5 Disable | While moving, safety officer presses Enter. | Robot stops. |
| 2.6 Fence | Shadow (D-pad down), hold trigger while the ghost goes back to its pickup (the target leaves the area). | Robot parks on the inner tape, center no more than ~0.15 m past it, never moving toward the outer tape. |
| 2.7 Contact | Line mirror, let the Cycler run into the robot. | Robot only gives way slowly (≤ 0.11 m/s at this cap) or slides along; rumble at 1.5 s; `BACKING_OFF` by 2.5 s; `PinViolations` stays 0. |

## Step 3: carpet, 1.0 m/s

`Assist speed cap` = **1.0**. Repeat 2.2–2.7. Then, one at a time, 60 s each:

- each mode with the D-pad (up Line mirror, right Cut-off, down Shadow, left Zone);
- Y (hold ground) with Shadow and with Line mirror;
- bumpers: focus #0, then back to auto;
- 3 ghosts, Evader; then the Choke layout.

Pass: same as step 2, with `CommandedSpeed` ≤ 1.0, stops on release within ~0.3 m, nothing past
the inner tape by more than ~0.15 m, `PinViolations` 0.

## What to log

- **Automatically:** the roboRIO `.wpilog` (all of `/PhantomTrack`, Driver Station state and
  joysticks). Copy it off the USB stick (or `/home/lvuser/logs`) at the end, plus the DS logs.
- **Trial log** (spotter), one line per trial: time, step #, cap, mode/hold, ghosts
  (count/type/layout), pass/fail, notes ("stopped 0.1 m past inner tape at the north side").
- **Video** from the spotter's position, with the trial number said out loud at the start.
- **Afterwards:** open the `.wpilog` in AdvantageScope with the layout. For each fail, find the
  moment and note `Status`, `PoseFault`, `CommandedSpeed`, `TargetDistance`.

## Known things you may see (not failures)

- `POSE_FAULT: POSITION_JUMP` right after Start or an alliance change: expected. Release and press
  the trigger again.
- The robot waits at the fence line after a pin back-off: the back-off pointed into the fence.
  Known open problem.
- `MaxPinSeconds` can pass 3.0 during a back-off at these low speeds. That timer counts any touch;
  `PinViolations` (contact while pushing in) is the one that matters and should stay 0.
