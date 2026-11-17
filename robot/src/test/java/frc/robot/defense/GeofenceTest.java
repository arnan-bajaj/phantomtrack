package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Vec2;

/**
 * Safety rule: geofence. Area [7.5, 12.5] x [1.5, 6.5], margin 0.75, braking 2.0 m/s^2.
 * Fence lines: x = 8.25 (west), x = 11.75 (east), y = 2.25 (south), y = 5.75 (north).
 */
class GeofenceTest {
  private static final double TOL = 1e-9;
  private static final Geofence FENCE = new Geofence(7.5, 1.5, 12.5, 6.5, 0.75, 2.0);

  @Test
  void farFromEveryWallNothingChanges() {
    // (10, 4): 1.75 m from each fence line in x and y -> v_max = sqrt(2 * 2 * 1.75) = 2.6458 m/s.
    assertEquals(Math.sqrt(7.0), FENCE.maxSpeedToward(1.75), TOL);
    Vec2 out = FENCE.limit(new Vec2(10.0, 4.0), new Vec2(1.0, -1.0));
    assertEquals(1.0, out.x(), TOL);
    assertEquals(-1.0, out.y(), TOL);
  }

  @Test
  void slowsDownOnTheWayToTheFence() {
    // x = 11.5: d_east = 11.75 - 11.5 = 0.25 -> v_max = sqrt(2 * 2 * 0.25) = 1.0
    assertEquals(1.0, FENCE.limit(new Vec2(11.5, 4.0), new Vec2(3.0, 0.0)).x(), TOL);
    // x = 11.625: d_east = 0.125 -> v_max = sqrt(2 * 2 * 0.125) = sqrt(0.5) = 0.70710678
    assertEquals(0.70710678, FENCE.limit(new Vec2(11.625, 4.0), new Vec2(3.0, 0.0)).x(), 1e-8);
    // y = 2.5: d_south = 2.5 - 2.25 = 0.25 -> v_max = 1.0, toward south is -y
    assertEquals(-1.0, FENCE.limit(new Vec2(10.0, 2.5), new Vec2(0.0, -3.0)).y(), TOL);
  }

  @Test
  void onTheFenceLineMotionTowardTheWallIsRemovedButSlidingIsKept() {
    // x = 11.75: d_east = 0 -> vx <= 0. (1.0, 0.5) -> (0, 0.5): slide along the fence.
    Vec2 out = FENCE.limit(new Vec2(11.75, 4.0), new Vec2(1.0, 0.5));
    assertEquals(0.0, out.x(), TOL);
    assertEquals(0.5, out.y(), TOL);
    // Driving away from the wall is untouched: d_west = 11.75 - 8.25 = 3.5 -> v_max 3.74 > 1.
    Vec2 away = FENCE.limit(new Vec2(11.75, 4.0), new Vec2(-1.0, 0.5));
    assertEquals(-1.0, away.x(), TOL);
    assertEquals(0.5, away.y(), TOL);
  }

  @Test
  void insideTheMarginOnlyMotionBackInIsAllowed() {
    // x = 12.0 is 0.25 m past the east fence line (d_east = -0.25): vx <= 0.
    assertEquals(0.0, FENCE.limit(new Vec2(12.0, 4.0), new Vec2(1.0, 0.0)).x(), TOL);
    assertEquals(-1.0, FENCE.limit(new Vec2(12.0, 4.0), new Vec2(-1.0, 0.0)).x(), TOL);
    // Corner (8.0, 2.0): past the west AND south lines. (-1, -1) -> (0, 0); (1, 1) unchanged.
    Vec2 into = FENCE.limit(new Vec2(8.0, 2.0), new Vec2(-1.0, -1.0));
    assertEquals(0.0, into.x(), TOL);
    assertEquals(0.0, into.y(), TOL);
    Vec2 out = FENCE.limit(new Vec2(8.0, 2.0), new Vec2(1.0, 1.0));
    assertEquals(1.0, out.x(), TOL);
    assertEquals(1.0, out.y(), TOL);
  }

  @Test
  void neverCommandsTowardAWallInsideTheMarginFromAnyDirection() {
    // Every heading at 4.5 m/s, at four depths into each margin band (line, 0.15, 0.45, 0.75 m).
    double[] depth = {0.0, 0.15, 0.45, 0.75};
    for (double d : depth) {
      for (int deg = 0; deg < 360; deg += 10) {
        Vec2 v = Vec2.polar(4.5, Math.toRadians(deg));
        assertTrue(FENCE.limit(new Vec2(11.75 + d, 4.0), v).x() <= 0.0, "east " + d + " " + deg);
        assertTrue(FENCE.limit(new Vec2(8.25 - d, 4.0), v).x() >= 0.0, "west " + d + " " + deg);
        assertTrue(FENCE.limit(new Vec2(10.0, 5.75 + d), v).y() <= 0.0, "north " + d + " " + deg);
        assertTrue(FENCE.limit(new Vec2(10.0, 2.25 - d), v).y() >= 0.0, "south " + d + " " + deg);
      }
    }
  }

  @Test
  void stoppingDistanceMatchesTheBrakeCurve() {
    // Drive east at the allowed speed, braking only as fast as the curve demands: the robot
    // must come to rest at the fence line (11.75). Start at 10.0 at 1.0 m/s.
    // In 20 ms steps the last step can cross the line slightly before v_max reaches 0:
    //   d left to the line, step = 0.02 * sqrt(2 * 2 * d) = 0.04 * sqrt(d)
    //   overshoot = 0.04 * sqrt(d) - d, largest at sqrt(d) = 0.02: 0.04 * 0.02 - 0.0004 = 0.0004 m
    // Once past the line v_max = 0, so it stops there: 0.4 mm into a 750 mm margin.
    double x = 10.0;
    for (int i = 0; i < 1000; i++) {
      double v = FENCE.limit(new Vec2(x, 4.0), new Vec2(1.0, 0.0)).x();
      x += v * 0.02;
      assertTrue(x <= 11.75 + 0.0004 + 1e-9, "passed the fence at step " + i + ": " + x);
    }
    assertEquals(11.75, x, 0.01); // and it does get there
  }

  @Test
  void containsIsTheOuterAreaEdgesIncluded() {
    assertTrue(FENCE.contains(new Vec2(7.5, 1.5)));
    assertTrue(FENCE.contains(new Vec2(12.5, 6.5)));
    assertTrue(FENCE.contains(new Vec2(12.0, 4.0))); // in the margin but still in the area
    assertFalse(FENCE.contains(new Vec2(7.49, 4.0)));
    assertFalse(FENCE.contains(new Vec2(10.0, 6.51)));
    assertFalse(FENCE.contains(new Vec2(Double.NaN, 4.0)));
    // An unset pose (odometry boots at 0, 0) is outside the first-test area.
    assertFalse(FENCE.contains(Vec2.ZERO));
  }

  @Test
  void nanInGivesStopOut() {
    assertEquals(Vec2.ZERO, FENCE.limit(new Vec2(Double.NaN, 4.0), new Vec2(1.0, 0.0)));
    assertEquals(Vec2.ZERO, FENCE.limit(new Vec2(10.0, 4.0), new Vec2(Double.NaN, 0.0)));
    assertEquals(
        Vec2.ZERO, FENCE.limit(new Vec2(10.0, 4.0), new Vec2(Double.POSITIVE_INFINITY, 0.0)));
  }

  @Test
  void areaIsCutDownToTheFieldAndAnAreaOffTheFieldIsRejected() {
    FieldConfig field = FieldConfig.placeholder(); // 16.54 x 8.07
    Geofence whole = Geofence.inside(field, -5.0, -5.0, 100.0, 100.0, 0.75, 2.0);
    assertEquals(0.0, whole.minX(), TOL);
    assertEquals(0.0, whole.minY(), TOL);
    assertEquals(16.54, whole.maxX(), TOL);
    assertEquals(8.07, whole.maxY(), TOL);
    // East fence line of the whole field: 16.54 - 0.75 = 15.79.
    assertEquals(15.79, whole.fenceCorners().get(1).x(), TOL);
    assertNull(Geofence.inside(field, 20.0, 1.0, 25.0, 6.0, 0.75, 2.0));
    assertNull(Geofence.inside(field, 9.0, 1.0, 8.0, 6.0, 0.75, 2.0)); // min > max
    assertNull(Geofence.inside(field, Double.NaN, 1.0, 8.0, 6.0, 0.75, 2.0));
  }

  @Test
  void fenceCornersAreTheAreaMinusTheMargin() {
    var c = FENCE.fenceCorners();
    assertEquals(new Vec2(8.25, 2.25), c.get(0));
    assertEquals(new Vec2(11.75, 2.25), c.get(1));
    assertEquals(new Vec2(11.75, 5.75), c.get(2));
    assertEquals(new Vec2(8.25, 5.75), c.get(3));
  }

  @Test
  void badConstructorArgumentsAreRejected() {
    assertThrows(IllegalArgumentException.class, () -> new Geofence(5, 1, 5, 6, 0.5, 2.0));
    assertThrows(IllegalArgumentException.class, () -> new Geofence(1, 1, 5, 6, -0.1, 2.0));
    assertThrows(IllegalArgumentException.class, () -> new Geofence(1, 1, 5, 6, 0.5, 0.0));
    assertThrows(IllegalArgumentException.class, () -> new Geofence(Double.NaN, 1, 5, 6, 0.5, 2.0));
  }
}
