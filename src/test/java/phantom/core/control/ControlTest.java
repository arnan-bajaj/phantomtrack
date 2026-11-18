package phantom.core.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseTarget;

class ControlTest {
  private static final double TOL = 1e-9;

  @Test
  void zeroErrorGivesFeedforwardOnly() {
    var c = new HolonomicController(3.0, 5.0, 4.5, 6.0);
    var target = new DefenseTarget(new Vec2(2, 2), 0.0, new Vec2(1, -1), 0);
    var cmd = c.calculate(new Pose(2, 2, 0.0), target);
    assertEquals(1.0, cmd.velocity().x(), TOL);
    assertEquals(-1.0, cmd.velocity().y(), TOL);
    assertEquals(0.0, cmd.omega(), TOL);
  }

  @Test
  void proportionalTermAndSpeedCap() {
    var c = new HolonomicController(3.0, 5.0, 4.5, 6.0);
    // Error 0.5 m in x -> 3 * 0.5 = 1.5 m/s, under the cap.
    var small = c.calculate(new Pose(0, 0, 0), new DefenseTarget(new Vec2(0.5, 0), 0, Vec2.ZERO, 0));
    assertEquals(1.5, small.velocity().x(), TOL);
    // Error 10 m -> 30 m/s requested, capped to 4.5.
    var big = c.calculate(new Pose(0, 0, 0), new DefenseTarget(new Vec2(10, 0), 0, Vec2.ZERO, 0));
    assertEquals(4.5, big.velocity().norm(), TOL);
  }

  @Test
  void headingTakesShortWayAndIsCapped() {
    var c = new HolonomicController(3.0, 5.0, 4.5, 6.0);
    // From 170 deg to -170 deg: +20 deg error -> 5 * 0.349 = 1.745 rad/s, positive.
    var cmd =
        c.calculate(
            new Pose(0, 0, Math.toRadians(170)),
            new DefenseTarget(Vec2.ZERO, Math.toRadians(-170), Vec2.ZERO, 0));
    assertEquals(5.0 * Math.toRadians(20), cmd.omega(), TOL);
    // 180 deg error -> 5 * pi = 15.7 rad/s requested, capped to 6.
    var capped = c.calculate(new Pose(0, 0, 0), new DefenseTarget(Vec2.ZERO, Math.PI, Vec2.ZERO, 0));
    assertEquals(6.0, capped.omega(), TOL);
  }

  @Test
  void blendGivesDriverControlWhenOffOrOverriding() {
    var blend = new DriverBlend(1.0, 0.5);
    var driverIdle = new ChassisCommand(new Vec2(0.1, 0), 0);
    var driverPushing = new ChassisCommand(new Vec2(3.0, 0), 0); // 3.0 > 0.5 * 4.5 = 2.25
    var assist = new ChassisCommand(new Vec2(0, 2), 1.0);

    assertSame(driverIdle, blend.blend(driverIdle, assist, false, 4.5, 6.0));
    assertSame(driverPushing, blend.blend(driverPushing, assist, true, 4.5, 6.0));
    var out = blend.blend(driverIdle, assist, true, 4.5, 6.0); // omega 0 <= 0.5 * 6: not overriding
    assertEquals(0.0, out.velocity().x(), TOL);
    assertEquals(2.0, out.velocity().y(), TOL);
  }

  @Test
  void turningIsAnOverrideToo() {
    // Audit M2: isOverriding only looked at |v|, so a driver who only turns never took over.
    // Overriding = |v| > f * maxSpeed or |omega| > f * maxOmega.
    var blend = DriverBlend.defaults(); // alpha 1, f 0.5
    double maxOmega = 2.0 * Math.PI; // f * maxOmega = 0.5 * 6.2831853 = 3.1415927 rad/s
    var assist = new ChassisCommand(new Vec2(1, 0), -1.0);
    var turning = new ChassisCommand(Vec2.ZERO, 4.0); // 4.0 > 3.1415927 -> override
    assertTrue(blend.isOverriding(turning, 4.5, maxOmega));
    assertSame(turning, blend.blend(turning, assist, true, 4.5, maxOmega));
    // 2.0 <= 3.1415927: inside the fraction, like a small translation: the assist keeps control
    var gentle = new ChassisCommand(Vec2.ZERO, 2.0);
    assertEquals(-1.0, blend.blend(gentle, assist, true, 4.5, maxOmega).omega(), TOL);
    // The robot's setting (alpha 1, f 0): any turn that survived the stick deadband takes over.
    var robot = new DriverBlend(1.0, 0.0);
    assertSame(gentle, robot.blend(gentle, assist, true, 1.0, 2.0));
    // Partial blend, alpha 0.25, f 0.9 (0.9 * 6.2831853 = 5.6548668): driver omega 100, no
    // translation, used to be blended in: 0.75 * 100 + 0.25 * (-1) = 74.75 rad/s, far over
    // maxOmega. 100 > 5.6548668 -> it is the driver's own command now.
    var partial = new DriverBlend(0.25, 0.9);
    var spin = new ChassisCommand(Vec2.ZERO, 100.0);
    assertSame(spin, partial.blend(spin, assist, true, 4.5, maxOmega));
    // Below the fraction the mix stays within the limits: 0.75 * 5 + 0.25 * (-1) = 3.5
    var five = new ChassisCommand(Vec2.ZERO, 5.0);
    assertEquals(3.5, partial.blend(five, assist, true, 4.5, maxOmega).omega(), TOL);
  }

  @Test
  void partialBlendMixesLinearly() {
    var blend = new DriverBlend(0.25, 0.9);
    var driver = new ChassisCommand(new Vec2(2, 0), 0.0);
    var assist = new ChassisCommand(new Vec2(0, 4), 2.0);
    var out = blend.blend(driver, assist, true, 4.5, 6.0); // omega 0 <= 0.9 * 6
    // 0.75 * (2,0) + 0.25 * (0,4) = (1.5, 1.0); omega 0.25 * 2 = 0.5
    assertEquals(1.5, out.velocity().x(), TOL);
    assertEquals(1.0, out.velocity().y(), TOL);
    assertEquals(0.5, out.omega(), TOL);
  }
}
