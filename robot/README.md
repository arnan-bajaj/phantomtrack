# PhantomTrack robot project (Phase 7)

PhantomTrack's defense assist running on a real swerve robot. It uses the robot's real pose,
and the opponents are **simulated "ghost" robots** shown in AdvantageScope. Nothing here does
localization: the pose comes from your drivetrain's pose estimator.

Before the first test on carpet, read [`../docs/phase7-test-plan.md`](../docs/phase7-test-plan.md).

## Build, test, simulate

Use WPILib 2026 (GradleRIO 2026.2.1, the version installed in `~/wpilib/2026`). From this folder:

```bash
export JAVA_HOME=~/wpilib/2026/jdk   # or use the WPILib VS Code terminal
./gradlew build                      # compiles + runs the unit tests (no robot needed)
./gradlew simulateJava               # desktop sim with the WPILib sim GUI
./gradlew deploy                     # needs your team number (WPILib: Set Team Number)
```

**Desktop sim by hand:** plug in an Xbox controller, start `simulateJava`, drag the controller
into Joystick slot 0 in the sim GUI, choose *Teleoperated*, and open AdvantageScope
(*File > Connect to Simulator*, then *File > Import Layout...* `advantagescope-layout.json`).
Hold the right trigger and the robot (the sim drivetrain) follows the active defense mode. The
keyboard works too, but the default keyboard map has no axis 3 (right trigger), so you would have
to add one in the GUI's keyboard settings.

**Desktop sim, scripted:** `tools/SimDriveCheck.java` plays the Driver Station and an Xbox
controller over the HAL WebSocket extension, so the same session can be repeated exactly:

```bash
./gradlew simulateJava -PsimHeadless | tee sim.log     # terminal 1 (no GUI, WebSocket on 3300)
java tools/SimDriveCheck.java                          # terminal 2 (~2 minutes)
```

The robot prints one `[PT]` status line per second in simulation. Check `sim.log` against the
"expect" text of each phase the script prints.

## How it is put together

```
../src/main/java/phantom/core/**      the defense library, compiled from the sim project (not copied)
../src/main/java/phantom/sim/<a few>  the sim's own opponent bots, reused as ghosts (no AWT/Swing)
src/main/java/frc/robot/defense/      robot logic WITHOUT WPILib: safety rules, ghosts, pipeline
src/main/java/frc/robot/              the thin WPILib layer: drivetrain, controller, dashboard, NT
```

- `shared-sources.txt` lists exactly which files are compiled from `../src/main/java`. The main
  project's `RobotSharedSourcesTest` fails if a listed sim file uses AWT, Swing, image IO, WPILib or
  the sim UI, and `CorePurityTest` keeps `phantom.core` free of all of those and of `phantom.sim`.
- `frc.robot.defense` has no WPILib imports (`DefensePackagePurityTest`). All decisions live there
  and are unit-tested: `./gradlew test` runs 76 tests without a robot or the HAL.
- Core always works in the **blue frame**. `AllianceFrame` mirrors poses on the way in and
  commands on the way out when the Driver Station says red (`FieldMirror`, both rotated and
  mirrored fields; set `Constants.MIRROR` from the 2027 manual).

One loop (20 ms), in `DefenseTeleop` (the drivetrain's default command):

```
pose, speeds, sticks (real frame) -> AllianceFrame.toBlue
  -> PhantomTrack.update: PoseMonitor -> SafetyGate.decide -> DefenseAssist (phantom.core)
                          -> VirtualContact -> SafetyGate.filter (cap -> accel -> geofence)
  -> AllianceFrame.commandToReal -> drivetrain.driveFieldRelative
  -> ghosts step against the real pose -> rumble + telemetry
```

## Safety rules

The assist only ever changes the robot's motion while ALL of these hold. Whenever the driver
drives, the driver's command goes to the drivetrain untouched.

| Rule | Where | Tests |
| --- | --- | --- |
| Assist only while the right trigger is held past 50% (deadman) | `SafetyGate` | `SafetyGateTest`, `PhantomTrackTest` |
| Any stick past the 0.1 deadband = driver override (same rule as `DriverBlend`, threshold 0) | `SafetyGate`, `DriverInput` | `SafetyGateTest`, `DriverInputTest`, `PhantomTrackTest` |
| Disabled in autonomous and test; robot is commanded to stop | `SafetyGate` | `SafetyGateTest`, `PhantomTrackTest` |
| Speed cap, default 1.0 m/s (max 4.5), turn cap 2 rad/s | `CommandLimits` | `CommandLimitsTest`, `PhantomTrackTest` |
| Acceleration limit, default 2 m/s^2 (and 6 rad/s^2) | `CommandLimits` | `CommandLimitsTest`, `PhantomTrackTest` |
| Geofence: test area minus 0.75 m; never commands toward a wall inside the margin, brakes before it | `Geofence` | `GeofenceTest`, `PhantomTrackTest` (60 s closed loop) |
| Pose stale (> 0.25 s), jumped (> 0.5 m or 0.5 rad in one loop), frozen while driving, or NaN: assist off until the pose is fine for 0.5 s AND the trigger is released | `PoseMonitor`, `SafetyGate` | `PoseMonitorTest`, `SafetyGateTest`, `PhantomTrackTest` |
| NaN can never reach the drivetrain | `CommandLimits` | several |
| Settings and ghost resets only apply with the trigger released | `PhantomTrack` | `PhantomTrackTest` |

The robot boots with the first-test values (`SafetyConfig.firstTest()`) every time. Nothing typed
on the dashboard survives a reboot. The **test area** defaults to a 5 x 5 m box,
x 7.5–12.5, y 1.5–6.5 (blue frame), around the defended goal. A robot whose pose was never set
sits at (0, 0), outside it, so the assist refuses to run until the pose is real.

## Driver controls (Xbox, port 0)

| Control | Action |
| --- | --- |
| Right trigger (hold) | assist deadman |
| Left stick / right stick X | drive (field-relative, 1.5 m/s at full stick by default); any movement takes control |
| D-pad up / right / down / left | Line mirror / Cut-off / Shadow / Zone |
| Right / left bumper | focus next / previous ghost (auto → #0 → #1 → ... → auto) |
| Y | hold ground on/off |
| Start | tell the pose estimator the robot is on the test start mark (trigger must be released) |
| Back | **simulation only**: make the reported pose jump 0.75 m |
| Rumble | touching a ghost for more than 1.5 s (the pin guard backs off at 2.5 s) |

## Dashboard (SmartDashboard/PhantomTrack/Settings)

Ghost count (0–3), ghost type (Cycler / Evader), field layout (Open / Choke / Two lanes, the
sim's made-up layouts), ghost max speed, assist speed cap, assist accel limit, driver max speed,
test area corners, fence margin, and a *Reset ghosts* button. Out-of-range numbers are pulled back
into range. Changes wait until the trigger is released (`SettingsPending` shows it).

## Telemetry (NetworkTables `/PhantomTrack`, recorded by DataLogManager)

Positions are in the real field frame, so ghosts appear where they are relative to the robot.

| Topic | Type |
| --- | --- |
| `RobotPose`, `BlueFrame/RobotPose` | Pose2d |
| `Ghosts`, `Target` (0 or 1 entries) | Pose2d[] |
| `Fence` (closed loop), `Goal` | Translation2d[] |
| `CommandedSpeeds` | ChassisSpeeds (field-relative) |
| `Status`, `Decision`, `Mode`, `Focus`, `Alliance`, `PoseFault` | string |
| `AssistDriving`, `Hold`, `PoseHealthy`, `PinWarning`, `SettingsPending` | boolean |
| `PinSeconds` (per ghost), `MaxPinSeconds`, `SpeedCap`, `TargetDistance`, `CommandedSpeed` | double / double[] |
| `GuardedGhost`, `PinViolations` | int |

`advantagescope-layout.json` (AdvantageScope 26.0.0, *File > Import Layout...*) has a 2D field
(robot, ghosts in magenta, target arrow, fence in yellow), a graph of pin timer / speed / target
distance with status stripes, and a status table.

## Ghosts

Ghosts are the simulator's own `CyclerBehavior` bots (Cycler drives through, Evader dodges with a
0.25 s human reaction delay) in a `SimWorld`. The only change is `SimWorld.stepWithRealSelf`: our
robot's state is measured, never simulated. Ghosts react to the real pose, can be blocked by the
real robot, and push each other as in the sim.

Contact with the real robot uses the sim's pushing-match model (`Contact`) on both sides:

- A ghost never overlaps the real robot (it gives up all of the overlap).
- While the **assist** drives, `VirtualContact` limits its motion into a touching ghost to the
  pair speed the sim would give: pushing a parked ghost flat out moves at 22% of the cap; equal
  pushing stalls; a ghost pushing harder makes the assist give way at that speed.
- While the **driver** drives, ghosts are only blocked. The driver's command is never changed.
- Pin time is shown per ghost and a referee count (`PinViolations`) uses the sim's rule:
  contact only counts while we are driving into the ghost.

## Wiring a real drivetrain

Implement `frc.robot.drive.DefenseDrive` and replace the two marked lines in
`RobotContainer` (`drive = ...; driveSubsystem = ...;`). Remove or unbind your normal teleop
drive command for the test: `DefenseTeleop` must be the drivetrain's default command, and it
already includes normal driving whenever the trigger is released. These snippets are not compiled
here (no vendor libraries in this build), so check them against your vendor library version.

### CTRE Tuner X swerve (Phoenix 6 generated `CommandSwerveDrivetrain`)

```java
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.drive.DefenseDrive;

public final class CtreDefenseDrive implements DefenseDrive {
  private final CommandSwerveDrivetrain drivetrain;
  // BlueAlliance perspective: PhantomTrack already sends real-field (blue-origin) velocities.
  // The generated code's default (OperatorPerspective) would flip them a second time on red.
  private final SwerveRequest.FieldCentric request =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity);

  public CtreDefenseDrive(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }

  @Override public ChassisSpeeds getFieldRelativeSpeeds() {
    var s = drivetrain.getState();
    return ChassisSpeeds.fromRobotRelativeSpeeds(s.Speeds, s.Pose.getRotation());
  }

  @Override public void driveFieldRelative(ChassisSpeeds v) {
    drivetrain.setControl(
        request
            .withVelocityX(v.vxMetersPerSecond)
            .withVelocityY(v.vyMetersPerSecond)
            .withRotationalRate(v.omegaRadiansPerSecond));
  }

  @Override public double getPoseTimestampSeconds() {
    // State.Timestamp is in CTRE's time base; convert to FPGA time.
    return Utils.currentTimeToFPGATime(drivetrain.getState().Timestamp);
  }

  @Override public boolean resetPose(Pose2d pose) {
    drivetrain.resetPose(pose);
    return true;
  }
}

// RobotContainer:  drive = new CtreDefenseDrive(drivetrain); driveSubsystem = drivetrain;
```

Written against the Phoenix 6 2025/2026 swerve API (`com.ctre.phoenix6.swerve`); older versions
used different package names. Keep the `DriveRequestType` your generated `RobotContainer` uses.

### YAGSL

```java
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.drive.DefenseDrive;
import swervelib.SwerveDrive;

public final class YagslDefenseDrive implements DefenseDrive {
  private final SwerveDrive swerve; // e.g. swerveSubsystem.getSwerveDrive()

  public YagslDefenseDrive(SwerveDrive swerve) {
    this.swerve = swerve;
  }

  @Override public Pose2d getPose() {
    return swerve.getPose();
  }

  @Override public ChassisSpeeds getFieldRelativeSpeeds() {
    return swerve.getFieldVelocity();
  }

  @Override public void driveFieldRelative(ChassisSpeeds v) {
    swerve.driveFieldOriented(v); // pure field frame, no alliance flip: what PhantomTrack wants
  }

  @Override public boolean resetPose(Pose2d pose) {
    swerve.resetOdometry(pose);
    return true;
  }
  // getPoseTimestampSeconds(): YAGSL does not expose when odometry last ran, so the default is
  // used and only the frozen-pose check protects against a dead odometry thread.
}

// RobotContainer:  drive = new YagslDefenseDrive(swerveSubsystem.getSwerveDrive());
//                  driveSubsystem = swerveSubsystem;
```

For both: if your AprilTag pipeline feeds the same pose estimator, a bad tag update shows up
here as a pose jump and switches the assist off (that is intended).

## Known limits

- Ghosts are only as good as the sim's bots (fixed routes, no game pieces). Use the results to
  check that the robot does what the sim does, not to measure defense.
- The pin guard's 0.5 s margin is part of `phantom.core` and assumes full speed. At the
  first-test caps (1.0 m/s, 2 m/s^2) backing off takes longer, and in the desktop sim the
  any-touch contact timer reached 3.2–3.7 s. The referee count (contact while pressing) stayed 0.
  A core change is proposed in the PR.
- Pin back-offs can point into the geofence; the robot then waits at the fence line until the
  back-off times out (same open problem as back-offs into walls in the sim).
- `getPoseTimestampSeconds()` defaults to "always fresh". Stale detection only works if the
  adapter reports a real timestamp (CTRE: yes; YAGSL: no, frozen-pose check only).

## WPILib 2027 / SystemCore: what will probably break

The robot layer is kept to a few small files on purpose. `phantom.core`, the ghost classes and
`frc.robot.defense` use no WPILib (enforced by tests), so they should move unchanged. Expect to
touch:

- **Build:** a new GradleRIO version and deploy target (SystemCore instead of `RoboRIO` in
  `build.gradle`), probably a newer JDK, and new vendordeps. Re-create the project from the 2027
  template and copy `build.gradle`'s shared-sources block over.
- **Package names and APIs:** WPILib has announced larger API changes for 2027 (including package
  renames). `Convert`, `AllianceFrame`, `DefenseTeleop`, `DefenseTelemetry`, `DefenseDashboard`,
  `RobotContainer`, `Robot`, `drive/` import WPILib; nothing else does.
- **Controllers:** HID classes and button/axis numbering may change with the new Driver Station.
  `DefenseTeleop` reads the sticks and trigger in one place; `RobotContainer` has the bindings.
- **Time:** `Timer.getFPGATimestamp()` assumes an FPGA clock. Use whatever 2027 provides for
  "monotonic robot time" in `DefenseTeleop`, `SimDefenseDrive` and `DefenseDrive`.
- **Simulation:** the HAL WebSocket extension and its message format (used by
  `tools/SimDriveCheck.java`) may change.
- **Vendor drivetrains:** the CTRE/YAGSL adapters above will need their 2027 versions.
- **Game:** `FieldConfig`, `PinRules`, `Constants.MIRROR` (rotated vs mirrored field) and the test
  area must come from the 2027 manual.
