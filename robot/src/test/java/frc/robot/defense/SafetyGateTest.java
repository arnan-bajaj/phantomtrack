package frc.robot.defense;

import static frc.robot.defense.SafetyGate.Decision.ASSIST;
import static frc.robot.defense.SafetyGate.Decision.DRIVER_OVERRIDE;
import static frc.robot.defense.SafetyGate.Decision.NOT_TELEOP;
import static frc.robot.defense.SafetyGate.Decision.OFF;
import static frc.robot.defense.SafetyGate.Decision.OUTSIDE_AREA;
import static frc.robot.defense.SafetyGate.Decision.POSE_FAULT;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Vec2;

/**
 * Safety rules: deadman, driver override, pose fault latch, test area, disabled outside teleop,
 * and the order the command filters run in. SafetyConfig.firstTest(): trigger threshold 0.5,
 * stick deadband 0.1, cap 1.0 m/s, 2.0 rad/s, accel 2.0 m/s^2, area [7.5, 12.5] x [1.5, 6.5],
 * margin 0.75 (fence lines x = 8.25 / 11.75, y = 2.25 / 5.75).
 */
class SafetyGateTest {
  private static final double TOL = 1e-9;
  private static final SafetyConfig CFG = SafetyConfig.firstTest();
  private static final Vec2 MIDDLE = new Vec2(10.0, 4.0);

  private static SafetyGate gate() {
    return new SafetyGate(
        CFG,
        Geofence.inside(
            phantom.core.field.FieldConfig.placeholder(),
            CFG.areaMinX(),
            CFG.areaMinY(),
            CFG.areaMaxX(),
            CFG.areaMaxY(),
            CFG.fenceMargin(),
            CFG.accelLimit()));
  }

  @Test
  void everythingRightGivesAssist() {
    assertEquals(ASSIST, gate().decide(RobotMode.TELEOP, 1.0, 0.0, true, MIDDLE));
  }

  @Test
  void deadmanMustBeHeldPastTheThreshold() {
    SafetyGate g = gate();
    assertEquals(OFF, g.decide(RobotMode.TELEOP, 0.0, 0.0, true, MIDDLE));
    assertEquals(OFF, g.decide(RobotMode.TELEOP, 0.5, 0.0, true, MIDDLE)); // needs > 0.5
    assertEquals(ASSIST, g.decide(RobotMode.TELEOP, 0.51, 0.0, true, MIDDLE));
    assertEquals(OFF, g.decide(RobotMode.TELEOP, Double.NaN, 0.0, true, MIDDLE));
  }

  @Test
  void anyStickPastTheDeadbandIsAnOverride() {
    SafetyGate g = gate();
    assertEquals(ASSIST, g.decide(RobotMode.TELEOP, 1.0, 0.1, true, MIDDLE)); // 0.1 <= 0.1
    assertEquals(DRIVER_OVERRIDE, g.decide(RobotMode.TELEOP, 1.0, 0.11, true, MIDDLE));
    assertEquals(DRIVER_OVERRIDE, g.decide(RobotMode.TELEOP, 1.0, 1.0, true, MIDDLE));
    assertEquals(DRIVER_OVERRIDE, g.decide(RobotMode.TELEOP, 1.0, Double.NaN, true, MIDDLE));
    // Letting go of the stick gives control straight back to the assist.
    assertEquals(ASSIST, g.decide(RobotMode.TELEOP, 1.0, 0.0, true, MIDDLE));
  }

  @Test
  void neverAssistsOutsideTeleop() {
    SafetyGate g = gate();
    for (RobotMode m : new RobotMode[] {RobotMode.AUTONOMOUS, RobotMode.DISABLED, RobotMode.TEST}) {
      assertEquals(NOT_TELEOP, g.decide(m, 1.0, 0.0, true, MIDDLE), m.toString());
    }
  }

  @Test
  void poseFaultLatchesUntilTheTriggerIsReleased() {
    SafetyGate g = gate();
    assertEquals(ASSIST, g.decide(RobotMode.TELEOP, 1.0, 0.0, true, MIDDLE));
    assertEquals(POSE_FAULT, g.decide(RobotMode.TELEOP, 1.0, 0.0, false, MIDDLE));
    // Pose is healthy again but the trigger is still held: stay off.
    assertEquals(POSE_FAULT, g.decide(RobotMode.TELEOP, 1.0, 0.0, true, MIDDLE));
    assertEquals(POSE_FAULT, g.decide(RobotMode.TELEOP, 1.0, 0.0, true, MIDDLE));
    // Release, then press again: assist.
    assertEquals(OFF, g.decide(RobotMode.TELEOP, 0.0, 0.0, true, MIDDLE));
    assertEquals(ASSIST, g.decide(RobotMode.TELEOP, 1.0, 0.0, true, MIDDLE));
  }

  @Test
  void releasingTheTriggerWhileThePoseIsStillBadDoesNotClearTheFault() {
    SafetyGate g = gate();
    g.decide(RobotMode.TELEOP, 1.0, 0.0, false, MIDDLE);
    assertEquals(OFF, g.decide(RobotMode.TELEOP, 0.0, 0.0, false, MIDDLE));
    assertEquals(POSE_FAULT, g.decide(RobotMode.TELEOP, 1.0, 0.0, false, MIDDLE));
  }

  @Test
  void outsideTheTestAreaTheDriverDrives() {
    SafetyGate g = gate();
    // Unset odometry (0, 0) is far outside [7.5, 12.5] x [1.5, 6.5].
    assertEquals(OUTSIDE_AREA, g.decide(RobotMode.TELEOP, 1.0, 0.0, true, Vec2.ZERO));
    assertEquals(OUTSIDE_AREA, g.decide(RobotMode.TELEOP, 1.0, 0.0, true, new Vec2(12.6, 4.0)));
    assertEquals(OUTSIDE_AREA, g.decide(RobotMode.TELEOP, 1.0, 0.0, true, null));
    // No valid area at all (e.g. typed off the field on the dashboard): never assist.
    SafetyGate none = new SafetyGate(CFG, null);
    assertEquals(OUTSIDE_AREA, none.decide(RobotMode.TELEOP, 1.0, 0.0, true, MIDDLE));
    assertSame(ChassisCommand.STOP, none.filter(new ChassisCommand(new Vec2(1, 0), 0), ChassisCommand.STOP, MIDDLE, 0.02));
  }

  @Test
  void filterCapsThenSlewsThenFences() {
    SafetyGate g = gate();
    // Want (3, 0) from rest: cap -> (1, 0); accel 2.0 * 0.02 = 0.04 -> (0.04, 0); fence at
    // (10, 4) allows sqrt(2 * 2 * 1.75) = 2.65 east -> (0.04, 0).
    ChassisCommand out =
        g.filter(new ChassisCommand(new Vec2(3, 0), 5.0), ChassisCommand.STOP, MIDDLE, 0.02);
    assertEquals(0.04, out.velocity().x(), TOL);
    assertEquals(0.0, out.velocity().y(), TOL);
    // omega: cap 5 -> 2; from 0 at 6 rad/s^2 * 0.02 = 0.12.
    assertEquals(0.12, out.omega(), TOL);
  }

  @Test
  void takingOverFromAFastDriverNeverExceedsTheCap() {
    // Driver was doing 2.0 m/s (no cap on the driver). Assist wants (1, 0): slew from (2, 0)
    // toward (1, 0) gives (1.96, 0), the second cap brings it to (1.0, 0).
    ChassisCommand out =
        gate()
            .filter(
                new ChassisCommand(new Vec2(1, 0), 0),
                new ChassisCommand(new Vec2(2, 0), 0),
                MIDDLE,
                0.02);
    assertEquals(1.0, out.velocity().x(), TOL);
  }

  @Test
  void fenceWinsOverEverythingElse() {
    // On the east fence line (x = 11.75) driving (0.8, 0.5): |v| = sqrt(0.64 + 0.25) = 0.943,
    // under the cap, and equal to last loop's command, so cap and slew leave it alone. The fence
    // then removes all of the eastward part at once, even though the accel limit alone would
    // only allow slowing by 0.04.
    ChassisCommand out =
        gate()
            .filter(
                new ChassisCommand(new Vec2(0.8, 0.5), 0),
                new ChassisCommand(new Vec2(0.8, 0.5), 0),
                new Vec2(11.75, 4.0),
                0.02);
    assertEquals(0.0, out.velocity().x(), TOL);
    assertEquals(0.5, out.velocity().y(), TOL);
  }

  @Test
  void nanAssistBecomesAControlledStop() {
    // NaN -> STOP, then the accel limit slows from the previous (0.5, 0) by 0.04: (0.46, 0).
    ChassisCommand out =
        gate()
            .filter(
                new ChassisCommand(new Vec2(Double.NaN, 0), 0),
                new ChassisCommand(new Vec2(0.5, 0), 0),
                MIDDLE,
                0.02);
    assertEquals(0.46, out.velocity().x(), TOL);
    assertEquals(true, out.velocity().isFinite() && Double.isFinite(out.omega()));
  }
}
