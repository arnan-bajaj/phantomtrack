package phantom.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.control.DriverBlend;
import phantom.core.control.HolonomicController;
import phantom.core.control.RulesGuard;
import phantom.core.field.FieldConfig;
import phantom.core.field.Obstacle;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.manager.DefenseAssist;
import phantom.core.modes.CutoffMode;
import phantom.core.modes.DefenseContext;
import phantom.core.modes.DefenseMode;
import phantom.core.modes.DefenseTarget;
import phantom.core.modes.HoldGroundMode;
import phantom.core.modes.InterposeMode;
import phantom.core.modes.LineMirrorMode;
import phantom.core.modes.ShadowMode;
import phantom.core.modes.ZoneMode;
import phantom.core.rules.DriveTelemetry;
import phantom.core.tracking.TrackedRobot;

/**
 * Seeded randomized invariants (plain loops, same numbers every run). Not hand-computed: they
 * check properties that must hold for every input, over thousands of random field-scale inputs.
 *
 * <ol>
 *   <li>DefenseAssist's command is always finite, and while the assist drives (engaged, driver
 *       not overriding) it is within the controller's speed and turn-rate limits.
 *   <li>Every mode's target (with and without hold ground), and the pin guard's output, is inside
 *       the field walls and outside every obstacle when we stand on a legal spot.
 *   <li>DriverBlend never ignores driver input above its override threshold (its deadband):
 *       driving OR turning past it gives the driver's own command; below it the driver still
 *       gets weight 1 - alpha.
 * </ol>
 *
 * Fields: the three sim layouts plus a 10 x 10 field with an obstacle on a field wall and one in
 * the middle (audit M3).
 */
class InvariantTest {
  private static final double EPS = 1e-9;

  private static List<FieldConfig> fields() {
    FieldConfig walls =
        new FieldConfig(
            "Wall test",
            10,
            10,
            new Vec2(9, 5),
            List.of(new Vec2(1.5, 1.0), new Vec2(1.5, 9.0)),
            0.45,
            List.of(Obstacle.of(0, 2, 1, 6), Obstacle.of(4, 4, 5, 5)),
            List.of());
    return List.of(FieldConfig.placeholder(), FieldConfig.choke(), FieldConfig.twoLanes(), walls);
  }

  private static List<DefenseMode> modes() {
    List<DefenseMode> base =
        List.of(
            ShadowMode.defaults(),
            InterposeMode.defaults(),
            ZoneMode.defaults(),
            LineMirrorMode.defaults(),
            CutoffMode.defaults());
    List<DefenseMode> all = new ArrayList<>(base);
    for (DefenseMode m : base) {
      all.add(HoldGroundMode.wrap(m));
    }
    return all;
  }

  /** Inside the walls (robot center at least halfSize from each) and out of every obstacle. */
  private static boolean legal(FieldConfig f, Vec2 p) {
    double r = f.robotHalfSize();
    if (!p.isFinite()
        || p.x() < r - EPS
        || p.x() > f.length() - r + EPS
        || p.y() < r - EPS
        || p.y() > f.width() - r + EPS) {
      return false;
    }
    for (Obstacle o : f.obstacles()) {
      if (o.blocks(p, r)) {
        return false;
      }
    }
    return true;
  }

  private static Vec2 randomPoint(Random rng, FieldConfig f) {
    return new Vec2(rng.nextDouble() * f.length(), rng.nextDouble() * f.width());
  }

  private static Vec2 randomVelocity(Random rng, double max) {
    return Vec2.polar(rng.nextDouble() * max, (rng.nextDouble() * 2 - 1) * Math.PI);
  }

  /** A legal spot: clampInside of a random point (itself checked: audit M3). */
  private static Vec2 legalPoint(Random rng, FieldConfig f) {
    Vec2 p = f.clampInside(randomPoint(rng, f));
    assertTrue(legal(f, p), "clampInside gave an illegal spot " + p + " on " + f.name());
    return p;
  }

  // ---------------- 1. commands ----------------

  @Test
  void commandsAreAlwaysFiniteAndWithinLimitsWhileTheAssistDrives() {
    Random rng = new Random(20261118L);
    List<FieldConfig> fields = fields();
    double dt = 0.02;
    int checked = 0;
    for (int trial = 0; trial < 300; trial++) {
      FieldConfig f = fields.get(rng.nextInt(fields.size()));
      double alpha = rng.nextDouble();
      double fraction = rng.nextDouble();
      HolonomicController controller = HolonomicController.defaults();
      int nOpp = rng.nextInt(4);
      List<Vec2> oppStart = new ArrayList<>();
      List<Vec2> oppVel = new ArrayList<>();
      for (int i = 0; i < nOpp; i++) {
        oppStart.add(legalPoint(rng, f));
        oppVel.add(randomVelocity(rng, 5.0));
      }
      double brokenChance = rng.nextDouble() < 0.3 ? 0.05 : 0.0;
      double[] clock = {0.0};
      DefenseAssist assist =
          new DefenseAssist(
              List.of(
                  ShadowMode.defaults(),
                  InterposeMode.defaults(),
                  ZoneMode.defaults(),
                  LineMirrorMode.defaults(),
                  CutoffMode.defaults()),
              controller,
              new DriverBlend(alpha, fraction),
              now -> {
                List<TrackedRobot> out = new ArrayList<>();
                for (int i = 0; i < nOpp; i++) {
                  double stamp = now - rng.nextDouble() * 0.3; // camera latency
                  Vec2 p = f.clampInside(oppStart.get(i).plus(oppVel.get(i).times(stamp)));
                  out.add(new TrackedRobot(1000 + i, p, oppVel.get(i), stamp, 1.0));
                }
                if (rng.nextDouble() < brokenChance) {
                  out.add(new TrackedRobot(7, new Vec2(Double.NaN, 1), Vec2.ZERO, now, 1.0));
                }
                return out;
              },
              f);
      assist.setModeIndex(rng.nextInt(5));
      assist.setHoldEnabled(rng.nextBoolean());
      assist.setFocusOpponentId(rng.nextBoolean() || nOpp == 0 ? -1 : 1000 + rng.nextInt(nOpp));
      Pose self = new Pose(legalPoint(rng, f), (rng.nextDouble() * 2 - 1) * Math.PI);
      Vec2 velocity = Vec2.ZERO;
      ChassisCommand last = ChassisCommand.STOP;
      for (int loop = 0; loop < 150; loop++) {
        double now = clock[0];
        boolean engaged = rng.nextDouble() < 0.9;
        // driver: mostly hands off, sometimes a small input, sometimes a real override
        ChassisCommand driver =
            switch (rng.nextInt(4)) {
              case 0 -> new ChassisCommand(randomVelocity(rng, 4.5), (rng.nextDouble() * 2 - 1) * 6);
              case 1 -> new ChassisCommand(randomVelocity(rng, 0.3), (rng.nextDouble() * 2 - 1) * 0.3);
              default -> ChassisCommand.STOP;
            };
        if (rng.nextBoolean()) {
          assist.setDriveTelemetry(
              new DriveTelemetry(
                  now, last.velocity(), velocity, randomVelocity(rng, 30.0), rng.nextDouble() * 80));
        }
        ChassisCommand cmd = assist.update(now, self, velocity, driver, engaged);
        assertTrue(cmd.velocity().isFinite() && Double.isFinite(cmd.omega()),
            "trial " + trial + " loop " + loop + ": " + cmd);
        boolean overriding =
            driver.velocity().norm() > fraction * controller.maxSpeed()
                || Math.abs(driver.omega()) > fraction * controller.maxOmega();
        if (engaged && !overriding) {
          assertTrue(cmd.velocity().norm() <= controller.maxSpeed() + EPS,
              "trial " + trial + " loop " + loop + " speed " + cmd.velocity().norm());
          assertTrue(Math.abs(cmd.omega()) <= controller.maxOmega() + EPS,
              "trial " + trial + " loop " + loop + " omega " + cmd.omega());
          checked++;
        }
        // a perfect drivetrain that the walls stop
        velocity = cmd.velocity();
        self = new Pose(f.clampInside(self.position().plus(velocity.times(dt))),
            phantom.core.geometry.Angles.wrap(self.heading() + cmd.omega() * dt));
        last = cmd;
        clock[0] += dt;
      }
    }
    assertTrue(checked > 20000, "too few assist-driven loops checked: " + checked);
  }

  // ---------------- 2. targets ----------------

  @Test
  void targetsAreInsideTheFieldAndOutsideObstacles() {
    Random rng = new Random(2027L);
    List<FieldConfig> fields = fields();
    List<DefenseMode> modes = modes();
    int checked = 0;
    for (int trial = 0; trial < 20000; trial++) {
      FieldConfig f = fields.get(rng.nextInt(fields.size()));
      DefenseMode mode = modes.get(rng.nextInt(modes.size()));
      Vec2 self = legalPoint(rng, f);
      List<TrackedRobot> opps = new ArrayList<>();
      int nOpp = 1 + rng.nextInt(3);
      for (int i = 0; i < nOpp; i++) {
        // often right next to us (contact, hold ground), otherwise anywhere legal
        Vec2 p =
            rng.nextBoolean()
                ? f.clampInside(self.plus(Vec2.polar(0.9 + rng.nextDouble() * 0.4,
                    (rng.nextDouble() * 2 - 1) * Math.PI)))
                : legalPoint(rng, f);
        opps.add(new TrackedRobot(i, p, randomVelocity(rng, 5.0), 0.0, 1.0));
      }
      int focus = rng.nextBoolean() ? -1 : rng.nextInt(nOpp);
      var ctx = new DefenseContext(0.0, new Pose(self, 0.0), Vec2.ZERO, opps, f, focus);
      Optional<DefenseTarget> t = mode.compute(ctx);
      if (t.isPresent()) {
        assertTrue(legal(f, t.get().position()),
            mode.name() + " on " + f.name() + ": target " + t.get().position() + " self " + self
                + " opps " + opps);
        assertTrue(Double.isFinite(t.get().heading()) && t.get().velocityFeedforward().isFinite());
        checked++;
      }
    }
    assertTrue(checked > 15000, "too few targets checked: " + checked);
  }

  @Test
  void thePinGuardsTargetsAreLegalToo() {
    // The guard replaces the mode's target while backing off (clamped retreat), yielding or
    // looking (our own spot). Opponents parked against us so that it does back off.
    Random rng = new Random(31L);
    List<FieldConfig> fields = fields();
    int backingOff = 0;
    for (int trial = 0; trial < 150; trial++) {
      FieldConfig f = fields.get(rng.nextInt(fields.size()));
      RulesGuard guard = RulesGuard.defaults(f);
      Vec2 self = legalPoint(rng, f);
      List<TrackedRobot> opps = new ArrayList<>();
      int nOpp = 1 + rng.nextInt(3);
      for (int i = 0; i < nOpp; i++) {
        Vec2 p = f.clampInside(self.plus(Vec2.polar(0.9, (rng.nextDouble() * 2 - 1) * Math.PI)));
        opps.add(new TrackedRobot(i, p, randomVelocity(rng, 1.0), 0.0, 1.0));
      }
      for (int loop = 0; loop < 200; loop++) {
        double now = loop * 0.02;
        List<TrackedRobot> seen = new ArrayList<>();
        for (TrackedRobot r : opps) {
          seen.add(new TrackedRobot(r.id(), r.position(), r.velocity(), now, 1.0));
        }
        Optional<DefenseTarget> out =
            guard.apply(now, new Pose(self, 0.0), seen, Optional.empty(),
                new DriveTelemetry(now, new Vec2(2, 0), Vec2.ZERO, Vec2.ZERO, 60));
        if (out.isPresent()) {
          assertTrue(legal(f, out.get().position()),
              "guard target " + out.get().position() + " self " + self + " on " + f.name());
          assertTrue(out.get().velocityFeedforward().isFinite() && Double.isFinite(out.get().heading()));
        }
        backingOff += guard.backingOff() ? 1 : 0;
      }
    }
    assertTrue(backingOff > 1000, "the guard hardly backed off: " + backingOff);
  }

  // ---------------- 3. driver input ----------------

  @Test
  void driverBlendNeverIgnoresInputAboveItsDeadband() {
    Random rng = new Random(7L);
    for (int trial = 0; trial < 100000; trial++) {
      double alpha = rng.nextDouble();
      double fraction = rng.nextDouble();
      double maxSpeed = 0.5 + rng.nextDouble() * 4.0;
      double maxOmega = 0.5 + rng.nextDouble() * 6.0;
      var blend = new DriverBlend(alpha, fraction);
      var driver = new ChassisCommand(randomVelocity(rng, maxSpeed),
          (rng.nextDouble() * 2 - 1) * maxOmega);
      var assist = new ChassisCommand(randomVelocity(rng, maxSpeed),
          (rng.nextDouble() * 2 - 1) * maxOmega);
      boolean engaged = rng.nextDouble() < 0.8;
      ChassisCommand out = blend.blend(driver, assist, engaged, maxSpeed, maxOmega);
      boolean above =
          driver.velocity().norm() > fraction * maxSpeed
              || Math.abs(driver.omega()) > fraction * maxOmega;
      if (!engaged || above) {
        assertSame(driver, out, "trial " + trial); // the driver's own command, untouched
      } else {
        // below the deadband: the mix, the driver's part weighted 1 - alpha, within the limits
        Vec2 v = driver.velocity().times(1 - alpha).plus(assist.velocity().times(alpha));
        assertEquals(v.x(), out.velocity().x(), 1e-12);
        assertEquals(v.y(), out.velocity().y(), 1e-12);
        assertEquals((1 - alpha) * driver.omega() + alpha * assist.omega(), out.omega(), 1e-12);
        assertTrue(out.velocity().norm() <= maxSpeed + 1e-9);
        assertTrue(Math.abs(out.omega()) <= maxOmega + 1e-9);
      }
    }
  }
}
