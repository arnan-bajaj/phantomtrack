package phantom.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.AllianceLabel;
import phantom.core.tracking.Detection;
import phantom.core.tracking.DetectionFrame;

/** Anonymous camera (occlusion, color labels, clutter, own-robot filter) and alliance partners. */
class AnonymousSensorTest {
  private static final double TOL = 1e-9;

  /** A robot that never moves: its "goal" is where it stands. */
  private static SimWorld.Opponent parked(int id, Vec2 p) {
    return new SimWorld.Opponent(RobotBody.typical(id, new Pose(p, 0)),
        new CyclerBehavior(p, p, 0.0, 1000.0));
  }

  private static SimWorld world(FieldConfig field, Pose self, List<Vec2> opps, List<Vec2> partners) {
    List<SimWorld.Opponent> o = new ArrayList<>();
    for (int i = 0; i < opps.size(); i++) {
      o.add(parked(i, opps.get(i)));
    }
    List<SimWorld.Opponent> p = new ArrayList<>();
    for (int i = 0; i < partners.size(); i++) {
      p.add(parked(200 + i, partners.get(i)));
    }
    return new SimWorld(field, RobotBody.typical(100, self), o, p);
  }

  /** Perfect camera (no noise, delay, dropouts, clutter, color errors), 70 deg, 8 m, 50 Hz. */
  private static AnonymousSensor.Config exact(boolean occlusion) {
    var cam = new NoisySensor.Config("test", 0.0, 0.0, 0.0, 70, 8.0, 50);
    return new AnonymousSensor.Config(cam, 0.0, 0.0, 0.5, 0.0, occlusion);
  }

  private static Detection at(List<Detection> ds, double x, double y) {
    for (Detection d : ds) {
      if (d.position().distanceTo(new Vec2(x, y)) < 1e-9) {
        return d;
      }
    }
    throw new AssertionError("no detection at (" + x + ", " + y + ") in " + ds);
  }

  // ---------------- occlusion ----------------

  @Test
  void occlusionGeometryByHand() {
    Vec2 cam = new Vec2(0, 0);
    Vec2 target = new Vec2(4, 0); // D = 4, u = (1, 0)
    double r = 0.45;
    // o = (2, 0.3): s = 2 (between), h = 0.3 < 0.45 -> hidden
    assertTrue(AnonymousSensor.occluded(cam, target, List.of(new Vec2(2, 0.3)), r));
    // o = (2, 0.5): h = 0.5 >= 0.45 -> the line passes beside it
    assertFalse(AnonymousSensor.occluded(cam, target, List.of(new Vec2(2, 0.5)), r));
    // o = (5, 0): s = 5 > D = 4 -> behind the target, not in the way
    assertFalse(AnonymousSensor.occluded(cam, target, List.of(new Vec2(5, 0)), r));
    // o = (-1, 0): s = -1 < 0 -> behind the camera
    assertFalse(AnonymousSensor.occluded(cam, target, List.of(new Vec2(-1, 0)), r));

    // Diagonal: camera (1,1), target (4,5): (3,4), D = 5, u = (0.6, 0.8)
    Vec2 c2 = new Vec2(1, 1);
    Vec2 t2 = new Vec2(4, 5);
    // o = (2.5, 3): o - c = (1.5, 2): s = 0.6*1.5 + 0.8*2 = 2.5, h = |0.6*2 - 0.8*1.5| = 0 -> hidden
    assertTrue(AnonymousSensor.occluded(c2, t2, List.of(new Vec2(2.5, 3)), r));
    // o = (3, 2): o - c = (2, 1): s = 1.2 + 0.8 = 2.0, h = |0.6*1 - 0.8*2| = 1.0 -> visible
    assertFalse(AnonymousSensor.occluded(c2, t2, List.of(new Vec2(3, 2)), r));
  }

  @Test
  void aRobotBehindAnotherIsNotSeen() {
    // We are at (5,4) facing +x. A at (8,4), B at (10, 4.2).
    // B: (5, 0.2), D = 5.004, u = (0.9992, 0.0400). A - camera = (3, 0):
    //   s = 3 * 0.9992 = 2.998 (between), h = |0.9992*0 - 0.0400*3| = 0.120 < 0.45 -> B hidden
    // A: D = 3; B - camera = (5, 0.2): s = 5 > 3 -> nothing in front of A
    var w = world(FieldConfig.placeholder(), new Pose(5, 4, 0),
        List.of(new Vec2(8, 4), new Vec2(10, 4.2)), List.of());
    var on = new AnonymousSensor(w, exact(true), 1);
    List<Detection> seen = on.poll(0.0).get(0).detections();
    assertEquals(1, seen.size());
    assertEquals(new Vec2(8, 4), seen.get(0).position());
    assertEquals(1, on.occludedRobots());
    // with occlusion off the same camera sees both
    var off = new AnonymousSensor(w, exact(false), 1);
    assertEquals(2, off.poll(0.0).get(0).detections().size());
  }

  // ---------------- labels, own robot ----------------

  @Test
  void partnersAreSeenTooWithAColorThatCanBeWrong() {
    // opponent at (8,4); partner at (8,6): rel (3,2), angle 33.7 deg < 35 deg -> inside the FOV
    var w = world(FieldConfig.placeholder(), new Pose(5, 4, 0),
        List.of(new Vec2(8, 4)), List.of(new Vec2(8, 6)));
    List<Detection> right = new AnonymousSensor(w, exact(true), 1).poll(0.0).get(0).detections();
    assertEquals(2, right.size());
    assertEquals(AllianceLabel.OPPONENT, at(right, 8, 4).label());
    assertEquals(AllianceLabel.PARTNER, at(right, 8, 6).label());
    // color error 1.0: every label is flipped
    var wrongCfg = exact(true).withColorError(1.0);
    var wrongSensor = new AnonymousSensor(w, wrongCfg, 1);
    List<Detection> wrong = wrongSensor.poll(0.0).get(0).detections();
    assertEquals(AllianceLabel.PARTNER, at(wrong, 8, 4).label());
    assertEquals(AllianceLabel.OPPONENT, at(wrong, 8, 6).label());
    assertEquals(2, wrongSensor.wrongLabels());
  }

  @Test
  void ourOwnRobotIsNeverReported() {
    // Nobody else on the field; the detector fires on our own bumpers every frame (no noise):
    // each one is inside our bumpers (distance 0 < 0.45) and removed.
    var w = world(FieldConfig.placeholder(), new Pose(5, 4, 0), List.of(), List.of());
    var s = new AnonymousSensor(w, exact(true).withSelfDetectionRate(1.0), 3);
    for (int i = 0; i < 100; i++) {
      for (DetectionFrame f : s.poll(i * 0.02)) {
        assertTrue(f.detections().isEmpty());
      }
    }
    assertEquals(100, s.frames());
    assertEquals(100, s.selfDetections());
    assertEquals(100, s.removedNearSelf());
  }

  // ---------------- clutter ----------------

  @Test
  void clutterHasThePoissonMeanAndStaysInTheFieldOfView() {
    // Big empty 40 x 40 field so no clutter falls outside it. Mean 2 per frame, 3000 frames:
    // the average should be 2 within a few standard errors (sqrt(2 / 3000) = 0.026).
    var field = new FieldConfig(40, 40, new Vec2(30, 20), List.of(), 0.45);
    var w = world(field, new Pose(20, 20, 0), List.of(), List.of());
    var s = new AnonymousSensor(w, exact(true).withClutter(2.0, 0.5), 11);
    int opp = 0;
    int reported = 0;
    for (int i = 0; i < 3000; i++) {
      for (DetectionFrame f : s.poll(i * 0.02)) {
        for (Detection d : f.detections()) {
          Vec2 rel = d.position().minus(new Vec2(20, 20));
          assertTrue(rel.norm() <= 8.0 + TOL, "beyond range: " + rel);
          assertTrue(Math.abs(Angles.wrap(rel.angle())) <= Math.toRadians(35) + TOL, "outside FOV");
          opp += d.label() == AllianceLabel.OPPONENT ? 1 : 0;
          reported++;
        }
      }
    }
    assertEquals(3000, s.frames());
    assertEquals(2.0, s.clutterDetections() / 3000.0, 0.1);
    assertEquals(0.5, (double) opp / reported, 0.03); // labels split 50/50
  }

  @Test
  void clutterOutsideTheFieldIsDropped() {
    // Facing the blue wall from 1 m away: most of the sector is outside the field.
    var w = world(FieldConfig.placeholder(), new Pose(1.0, 4.035, Math.PI), List.of(), List.of());
    var s = new AnonymousSensor(w, exact(true).withClutter(5.0, 0.5), 5);
    int reported = 0;
    for (int i = 0; i < 500; i++) {
      for (DetectionFrame f : s.poll(i * 0.02)) {
        for (Detection d : f.detections()) {
          assertTrue(d.position().x() >= 0 && d.position().x() <= 16.54);
          assertTrue(d.position().y() >= 0 && d.position().y() <= 8.07);
          reported++;
        }
      }
    }
    assertTrue(reported < 500 * 5 / 2, "most clutter should be outside the field: " + reported);
  }

  @Test
  void persistentFalsePositivesRepeatInTheSamePlace() {
    // 5 spots, detected every frame (rate 1), no noise, 360 deg view with 50 m range so all are in
    // view, nobody to hide them: every frame has the same 5 positions with the same colors.
    var field = FieldConfig.placeholder();
    var w = world(field, new Pose(8, 4, 0), List.of(), List.of());
    var cam = new NoisySensor.Config("test", 0.0, 0.0, 0.0, 360, 50.0, 50);
    var cfg = new AnonymousSensor.Config(cam, 0.0, 0.0, 0.5, 0.0, true)
        .withPersistentFalsePositives(5, 1.0);
    var s = new AnonymousSensor(w, cfg, 4);
    List<Vec2> spots = s.persistentSpots();
    assertEquals(5, spots.size());
    for (Vec2 p : spots) { // placed inside the field, at least 0.5 m from the walls
      assertTrue(p.x() >= 0.5 && p.x() <= field.length() - 0.5, p.toString());
      assertTrue(p.y() >= 0.5 && p.y() <= field.width() - 0.5, p.toString());
    }
    List<Detection> first = s.poll(0.0).get(0).detections();
    assertEquals(5, first.size());
    for (int i = 1; i < 50; i++) {
      List<Detection> next = s.poll(i * 0.02).get(0).detections();
      assertEquals(5, next.size());
      for (Detection d : next) { // same place, same color as in the first frame
        assertEquals(at(first, d.position().x(), d.position().y()).label(), d.label());
      }
    }
    assertEquals(250, s.phantomDetections());
    // the same seed puts the spots in the same places
    assertEquals(spots, new AnonymousSensor(w, cfg, 4).persistentSpots());
  }

  @Test
  void persistentFalsePositivesFireAtTheirRate() {
    // one spot, 30% per frame, 2000 frames in view: about 600 detections (sd = sqrt(2000*.3*.7) = 20)
    var w = world(FieldConfig.placeholder(), new Pose(8, 4, 0), List.of(), List.of());
    var cam = new NoisySensor.Config("test", 0.0, 0.0, 0.0, 360, 50.0, 50);
    var s = new AnonymousSensor(w, new AnonymousSensor.Config(cam, 0.0, 0.0, 0.5, 0.0, true)
        .withPersistentFalsePositives(1, 0.3), 8);
    for (int i = 0; i < 2000; i++) {
      s.poll(i * 0.02);
    }
    assertEquals(600, s.phantomDetections(), 80);
  }

  // ---------------- timing ----------------

  @Test
  void framesArriveLateStampedWithCaptureTimeEvenWhenEmpty() {
    var cam = new NoisySensor.Config("test", 0.0, 0.1, 0.0, 70, 8.0, 50);
    var w = world(FieldConfig.placeholder(), new Pose(5, 4, 0), List.of(), List.of());
    var s = new AnonymousSensor(w, new AnonymousSensor.Config(cam, 0, 0, 0.5, 0, true), 1);
    assertTrue(s.poll(0.0).isEmpty()); // captured at 0.0, due at 0.1
    assertTrue(s.poll(0.06).isEmpty());
    List<DetectionFrame> got = s.poll(0.1);
    assertEquals(1, got.size());
    assertEquals(0.0, got.get(0).timestampSec(), TOL);
    assertTrue(got.get(0).detections().isEmpty()); // nothing seen, still a frame
  }

  @Test
  void sameSeedSameDetections() {
    var w = world(FieldConfig.placeholder(), new Pose(5, 4, 0),
        List.of(new Vec2(8, 4), new Vec2(9, 3)), List.of(new Vec2(8, 6)));
    var a = new AnonymousSensor(w, AnonymousSensor.Config.bad(), 9);
    var b = new AnonymousSensor(w, AnonymousSensor.Config.bad(), 9);
    for (int i = 0; i < 200; i++) {
      assertEquals(a.poll(i * 0.02), b.poll(i * 0.02));
    }
  }

  // ---------------- partners ----------------

  @Test
  void partnersAreOptionalMirroredAndNeverOpponents() {
    var field = FieldConfig.placeholder();
    SimWorld without = Scenarios.three(field, Scenarios.OpponentKind.CYCLER);
    assertTrue(without.partners().isEmpty());
    SimWorld with = Scenarios.three(field, Scenarios.OpponentKind.CYCLER,
        Scenarios.DEFENDER_START, true);
    assertEquals(3, with.truth().size()); // opponents only
    assertEquals(2, with.partners().size());
    // pickups mirrored through x = 16.54 / 2: (1.5, 1.2) -> (16.54 - 1.5, 1.2) = (15.04, 1.2)
    var p = with.partnerTruth();
    assertEquals(Scenarios.FIRST_PARTNER_ID, p.get(0).id());
    assertEquals(15.04, p.get(0).position().x(), TOL);
    assertEquals(1.2, p.get(0).position().y(), TOL);
    assertEquals(15.04, p.get(1).position().x(), TOL);
    assertEquals(6.9, p.get(1).position().y(), TOL);
    // our goal = mirror of the defended goal: (16.54 - 13.0, 4.035) = (3.54, 4.035)
    Vec2 ourGoal = Scenarios.mirror(field, field.defendedGoal());
    assertEquals(3.54, ourGoal.x(), TOL);
    assertEquals(4.035, ourGoal.y(), TOL);
  }

  @Test
  void partnersCycleToOurGoalAndDoNotCountAsOpponentContact() {
    var field = FieldConfig.placeholder();
    SimWorld w = Scenarios.three(field, Scenarios.OpponentKind.CYCLER,
        Scenarios.parkedOutOfTheWay(field), true);
    for (int i = 0; i < 3000; i++) { // 60 s, our robot parked in a corner
      w.step(ChassisCommand.STOP, 0.02);
    }
    for (SimWorld.Opponent partner : w.partners()) {
      assertTrue(partner.behavior().cyclesCompleted() >= 3,
          "partner cycles: " + partner.behavior().cyclesCompleted());
    }
    assertEquals(0, w.pinViolations());
  }
}
