package phantom.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.rules.DriveTelemetry;

/** DC motor model, per-module currents, measured acceleration and the telemetry sampler. */
class DriveTelemetryTest {
  private static final double TOL = 1e-9;
  private static final DriveMotorModel KRAKEN = DriveMotorModel.krakenX60();

  // ---------------- motor constants ----------------

  @Test
  void krakenConstantsByHand() {
    // R = 12 / 366 = 0.0327869 ohm
    assertEquals(0.0327869, KRAKEN.resistanceOhms(), 1e-7);
    // free speed = 6000 rpm * 2 pi / 60 = 628.3185 rad/s
    // Kv = 628.3185 / (12 - 0.0327869 * 2) = 628.3185 / 11.9344262 = 52.64757 rad/s per volt
    assertEquals(52.64757, KRAKEN.kvRadPerSecPerVolt(), 1e-5);
    // 4.5 m/s at the wheel: 4.5 / 0.0508 * 6.75 = 597.93307 rad/s at the motor
    assertEquals(597.93307, KRAKEN.motorSpeed(4.5), 1e-5);
  }

  @Test
  void neoConstantsByHand() {
    DriveMotorModel neo = DriveMotorModel.neo();
    // R = 12 / 105 = 0.1142857 ohm
    assertEquals(0.1142857, neo.resistanceOhms(), 1e-7);
    // free speed = 5676 rpm * 2 pi / 60 = 594.38933 rad/s
    // Kv = 594.38933 / (12 - 0.1142857 * 1.8) = 594.38933 / 11.7942857 = 50.39638
    assertEquals(50.39638, neo.kvRadPerSecPerVolt(), 1e-5);
  }

  // ---------------- current from the motor model ----------------

  @Test
  void cruisingAtTheCommandedSpeedDrawsNothing() {
    // omega = omega_cmd: V = omega/Kv + Kp*0, I = (omega/Kv - omega/Kv) / R = 0
    assertEquals(0.0, KRAKEN.moduleCurrent(4.5, 4.5), TOL);
    assertEquals(0.0, KRAKEN.moduleCurrent(0.0, 0.0), TOL);
  }

  @Test
  void stalledWheelHitsTheCurrentLimit() {
    // asked for 4.5 m/s, wheel not turning: omega_cmd = 597.933, omega = 0
    // V = 597.933/52.64757 + 0.016*597.933 = 11.357 + 9.567 = 20.92 -> battery limit 12 V
    assertEquals(12.0, KRAKEN.appliedVolts(597.93307, 0.0), TOL);
    // I = (12 - 0) / 0.0327869 = 366 A -> current limit 60 A
    assertEquals(60.0, KRAKEN.moduleCurrent(4.5, 0.0), TOL);
    // even a gentle 0.5 m/s push: omega_cmd = 0.5/0.0508*6.75 = 66.437
    // V = 66.437/52.64757 + 0.016*66.437 = 1.26192 + 1.06299 = 2.32491
    // I = 2.32491 / 0.0327869 = 70.91 A -> 60 A
    assertEquals(2.32491, KRAKEN.appliedVolts(66.43701, 0.0), 1e-5);
    assertEquals(60.0, KRAKEN.moduleCurrent(0.5, 0.0), TOL);
  }

  @Test
  void smallSpeedErrorGivesProportionalCurrent() {
    // asked 1.0 m/s, rolling 0.9: omega_cmd = 132.87402, omega = 119.58661, error 13.28740
    // V = 132.87402/52.64757 + 0.016*13.28740 = 2.52384 + 0.21260 = 2.73644
    // I = (2.73644 - 119.58661/52.64757) / 0.0327869 = (2.73644 - 2.27146) / 0.0327869 = 14.182 A
    assertEquals(14.182, KRAKEN.moduleCurrent(1.0, 0.9), 1e-3);
    // rolling FASTER than asked (braking): asked 4.0, rolling 4.2 -> error -26.5748
    // I = error * (1/Kv + Kp) / R = -26.5748 * (0.0189942 + 0.016) / 0.0327869 = -28.364 A
    assertEquals(-28.364, KRAKEN.moduleCurrent(4.0, 4.2), 1e-3);
  }

  @Test
  void neoStallCurrentIsLowerAtLowSpeed() {
    // NEO asked 0.5 m/s, stalled: V = 66.437/50.39638 + 0.016*66.437 = 1.31829 + 1.06299 = 2.38128
    // I = 2.38128 / 0.1142857 = 20.836 A (below its 40 A limit; the Kraken would be at 60 A)
    assertEquals(20.836, DriveMotorModel.neo().moduleCurrent(0.5, 0.0), 1e-3);
  }

  // ---------------- per-module currents ----------------

  @Test
  void pureTranslationLoadsAllModulesEqually() {
    // every module is asked for 4.5 m/s and rolls at 0: 60 A each (see stall test)
    double[] i = DriveTelemetrySim.moduleCurrents(KRAKEN, new Vec2(4.5, 0), 0, Vec2.ZERO, 0, 0.7);
    assertEquals(4, i.length);
    for (double amps : i) {
      assertEquals(60.0, amps, TOL);
    }
  }

  @Test
  void spinningWhileAskedToStopBrakesEveryModule() {
    // v = 0, omega = 1 rad/s, heading 0. Front-left module at r = (0.27, 0.27):
    // v_i = omega * perp(r) = (-0.27, 0.27), |v_i| = 0.381838 m/s, asked to stop (s_cmd = 0)
    // omega_motor = 0.381838/0.0508*6.75 = 50.7363
    // V = 0.016 * (0 - 50.7363) = -0.81178
    // I = (-0.81178 - 50.7363/52.64757) / 0.0327869 = (-0.81178 - 0.96370) / 0.0327869 = -54.152 A
    double[] i = DriveTelemetrySim.moduleCurrents(KRAKEN, Vec2.ZERO, 0, Vec2.ZERO, 1.0, 0);
    for (double amps : i) {
      assertEquals(-54.152, amps, 1e-3);
    }
  }

  @Test
  void rotationMakesModulesDiffer() {
    // asked (2,0) with no turn, actually moving (2,0) while spinning at 1 rad/s, heading 0
    // front-left  r = (0.27, 0.27): v_i = (2,0) + (-0.27, 0.27) = (1.73, 0.27), s = 1.73
    //   wheel error 0.27 m/s = 35.876 rad/s at the motor
    //   I = 35.876 * (1/52.64757 + 0.016) / 0.0327869 = 35.876 * 1.067312 = +38.291 A
    // front-right r = (0.27,-0.27): v_i = (2,0) + (0.27, 0.27) = (2.27, 0.27), s = 2.27 -> -38.291 A
    double[] i =
        DriveTelemetrySim.moduleCurrents(KRAKEN, new Vec2(2, 0), 0, new Vec2(2, 0), 1.0, 0);
    assertEquals(38.291, i[0], 1e-3); // front-left
    assertEquals(-38.291, i[1], 1e-3); // front-right
    assertEquals(38.291, i[2], 1e-3); // rear-left   r = (-0.27, 0.27): same perp x as front-left
    assertEquals(-38.291, i[3], 1e-3); // rear-right
  }

  // ---------------- measured acceleration ----------------

  @Test
  void bodyReportsItsAcceleration() {
    var field = FieldConfig.placeholder();
    var body = new RobotBody(0, new Pose(5, 4, 0), 4.5, 6.0, 6.0, 12.0);
    body.step(new ChassisCommand(new Vec2(4.5, 0), 0), 0.02, field);
    // limited to 6 m/s^2: v goes 0 -> 0.12 in 0.02 s, a = 0.12 / 0.02 = 6
    assertEquals(6.0, body.acceleration().x(), TOL);
    assertEquals(0.0, body.acceleration().y(), TOL);
  }

  @Test
  void hittingAWallIsAHugeDeceleration() {
    var field = FieldConfig.placeholder();
    // starts 0.05 m from the left wall limit (x = 0.45), accel limit high enough to be at -4.5 m/s
    // after one step: step 1 moves 0.09 m -> clamped to 0.45, velocity into the wall zeroed
    var body = new RobotBody(0, new Pose(0.5, 4, 0), 4.5, 1000.0, 6.0, 12.0);
    body.step(new ChassisCommand(new Vec2(-4.5, 0), 0), 0.02, field);
    assertEquals(0.45, body.position().x(), TOL);
    // a = (0 - 0) / 0.02 = 0 over the whole step (it started and ended at rest) ...
    assertEquals(0.0, body.acceleration().x(), TOL);
    // ... so drive away first, then come back fast: now the wall takes -4.5 m/s to 0 in one step
    var runner = new RobotBody(0, new Pose(3.0, 4, 0), 4.5, 6.0, 6.0, 12.0);
    Vec2 before = Vec2.ZERO;
    double peak = 0.0;
    for (int i = 0; i < 100; i++) {
      before = runner.velocity();
      runner.step(new ChassisCommand(new Vec2(-4.5, 0), 0), 0.02, field);
      peak = Math.max(peak, runner.acceleration().x());
      if (runner.position().x() == 0.45) {
        break;
      }
    }
    // 3.0 -> 0.45 is 2.55 m; from rest at 6 m/s^2 it reaches 4.5 m/s after 1.6875 m, so it
    // arrives at -4.5 m/s: a = (0 - (-4.5)) / 0.02 = +225 m/s^2
    assertEquals(-4.5, before.x(), TOL);
    assertEquals(225.0, peak, 1e-6);
  }

  // ---------------- the sampler ----------------

  @Test
  void samplesEveryPeriodAndHoldsInBetween() {
    var body = RobotBody.typical(100, new Pose(5, 4, 0));
    var sim = new DriveTelemetrySim(body, KRAKEN, DriveTelemetrySim.Config.noiseless(), 1);
    DriveTelemetry a = sim.sample(0.0);
    assertSame(a, sim.sample(0.01)); // 10 ms later: still the 0.0 s sample
    DriveTelemetry b = sim.sample(0.02);
    assertNotSame(a, b);
    assertEquals(0.02, b.timestampSec(), TOL);
  }

  @Test
  void stalledPushShowsUpInTheTelemetry() {
    // Head-on equal push (same setup as ContactAndAvoidanceTest): after 3 s both are stalled.
    var field = FieldConfig.placeholder();
    OpponentBehavior pushBack =
        new OpponentBehavior() {
          public String name() {
            return "Scripted";
          }

          public ChassisCommand decide(double t, double dt, RobotBody me, SimWorld w) {
            return new ChassisCommand(new Vec2(-4.5, 0), 0);
          }

          public int cyclesCompleted() {
            return 0;
          }

          public double averageCycleSeconds() {
            return Double.NaN;
          }
        };
    var world =
        new SimWorld(
            field,
            RobotBody.typical(100, new Pose(7.6, 4, 0)),
            List.of(new SimWorld.Opponent(RobotBody.typical(0, new Pose(8.5, 4, Math.PI)), pushBack)));
    var sim = new DriveTelemetrySim(world.self(), KRAKEN, DriveTelemetrySim.Config.noiseless(), 1);
    for (int i = 0; i < 150; i++) {
      world.step(new ChassisCommand(new Vec2(4.5, 0), 0), 0.02);
    }
    DriveTelemetry t = sim.sample(world.timeSec());
    assertEquals(4.5, t.commandedVelocity().x(), TOL); // still asking for full speed
    assertEquals(0.0, t.measuredVelocity().x(), 1e-6); // net effort 0 -> contact speed 0
    assertEquals(0.0, t.measuredAcceleration().x(), 1e-6); // and not gaining any
    assertEquals(60.0, t.driveCurrentAmps(), TOL); // every module at the current limit
    assertTrue(world.selfTouchingNow().contains(0));
    assertTrue(world.selfPressingNow().contains(0));
  }

  @Test
  void noiseIsSeededAndRoughlyTheRequestedSize() {
    var body = RobotBody.typical(100, new Pose(5, 4, 0)); // parked: true values are all zero
    var cfg = new DriveTelemetrySim.Config(0.02, 0.03, 0.5, 2.0);
    var a = new DriveTelemetrySim(body, KRAKEN, cfg, 7);
    var b = new DriveTelemetrySim(body, KRAKEN, cfg, 7);
    double sumV = 0;
    double sumA = 0;
    int n = 2000;
    for (int i = 0; i < n; i++) {
      DriveTelemetry ta = a.sample(i * 0.02);
      DriveTelemetry tb = b.sample(i * 0.02);
      assertEquals(ta, tb); // same seed, same samples
      sumV += ta.measuredVelocity().x() * ta.measuredVelocity().x();
      sumA += ta.measuredAcceleration().x() * ta.measuredAcceleration().x();
      assertTrue(ta.driveCurrentAmps() >= 0.0);
    }
    assertEquals(0.03, Math.sqrt(sumV / n), 0.003); // within 10% of the requested std
    assertEquals(0.5, Math.sqrt(sumA / n), 0.05);
  }
}
