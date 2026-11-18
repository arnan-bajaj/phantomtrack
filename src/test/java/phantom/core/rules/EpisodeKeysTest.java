package phantom.core.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/** EpisodeKeys with rebind radius 1.0 m and freshness 0.25 s (RulesGuard's values). */
class EpisodeKeysTest {
  private static final double TOL = 1e-9;
  private static final int UNKNOWN = ContactDetector.UNKNOWN;
  private static final int S = EpisodeKeys.SYNTHETIC_START;

  private static EpisodeKeys keys() {
    return new EpisodeKeys(1.0, 0.25, true);
  }

  private static TrackedRobot r(int id, double x, double y, double t) {
    return new TrackedRobot(id, new Vec2(x, y), Vec2.ZERO, t, 1.0);
  }

  private static List<Integer> ids(List<TrackedRobot> keyed) {
    return keyed.stream().map(TrackedRobot::id).toList();
  }

  @Test
  void stableIdsAreTheirOwnKeys() {
    var k = keys();
    var out = k.rekey(0.0, List.of(r(2, 1, 1, 0), r(0, 3, 3, 0)), id -> false);
    assertEquals(List.of(2, 0), ids(out)); // same order as the input
    assertEquals(new Vec2(1, 1), out.get(0).position());
    assertEquals(2, k.keyOf(2));
    assertEquals(2, k.aliasOf(2));
    assertEquals(UNKNOWN, k.keyOf(UNKNOWN));
    assertEquals(-2, k.keyOf(99)); // never seen
  }

  @Test
  void newIdNearALostOpenEpisodeTakesItOver() {
    var k = keys();
    k.rekey(0.0, List.of(r(1000, 5.9, 5, 0.0)), id -> true);
    // #1000 gone, #1001 at (6.5,5.8): d = |(0.6, 0.8)| = 1.0 <= 1.0 -> takes over key 1000
    var out = k.rekey(0.1, List.of(r(1001, 6.5, 5.8, 0.1)), id -> id == 1000);
    assertEquals(List.of(1000), ids(out));
    assertEquals(1000, k.keyOf(1001));
    assertEquals(1001, k.aliasOf(1000));
    assertEquals(1, k.rebinds());
  }

  @Test
  void tooFarOrNoOpenEpisodeMeansANewKey() {
    var k = keys();
    k.rekey(0.0, List.of(r(1000, 5.9, 5, 0.0)), id -> true);
    // d = |(0.61, 0.8)| = sqrt(0.3721 + 0.64) = 1.00603 > 1.0 -> own key (its id)
    var out = k.rekey(0.1, List.of(r(1001, 6.51, 5.8, 0.1)), id -> id == 1000);
    assertEquals(List.of(1001), ids(out));
    // Close, but #1000's episode is closed: nothing to continue.
    var k2 = keys();
    k2.rekey(0.0, List.of(r(1000, 5.9, 5, 0.0)), id -> false);
    assertEquals(List.of(1001), ids(k2.rekey(0.1, List.of(r(1001, 5.9, 5, 0.1)), id -> false)));
    assertEquals(0, k.rebinds() + k2.rebinds());
  }

  @Test
  void aStaleTrackLosesItsEpisodeToANewTrackOnTheSameRobot() {
    var k = keys();
    k.rekey(0.0, List.of(r(1000, 5.9, 5, 0.0)), id -> true);
    // t = 0.5: #1000 still reported but its estimate is from t = 0.2 (age 0.3 > 0.25: stale);
    // new #1001 at (6.0,5): d = 0.1 from #1000's last fresh position (5.9,5).
    var out = k.rekey(0.5, List.of(r(1000, 5.9, 5, 0.2), r(1001, 6.0, 5, 0.5)), id -> id == 1000);
    // #1001 continues the episode (key 1000); stale #1000 becomes its own episode: the id 1000 is
    // taken as a key, so it gets the first synthetic key.
    assertEquals(List.of(S, 1000), ids(out));
    assertEquals(1000, k.keyOf(1001));
    assertEquals(S, k.keyOf(1000));
  }

  @Test
  void beforeAnyFreshSightingTheLastReportedPositionCounts() {
    // Audit L3. #5 is only ever reported stale (age 0.5 > 0.25): at (2,2) at t = 1.0, then at
    // (2.9,2) at t = 1.1. Before any fresh sighting lastPos(5) must be its LAST reported position
    // (2.9,2) (class comment); the code kept the first one, (2,2).
    // At t = 1.2 new #6 appears at (3.85,2): |(3.85,2) - (2.9,2)| = 0.95 <= 1.0 -> it continues
    // #5's open episode (key 5); from (2,2) it would be 1.85 > 1.0, a new key.
    var k = keys();
    k.rekey(1.0, List.of(r(5, 2, 2, 0.5)), id -> id == 5);
    k.rekey(1.1, List.of(r(5, 2.9, 2, 0.6)), id -> id == 5);
    var out = k.rekey(1.2, List.of(r(5, 2.9, 2, 0.7), r(6, 3.85, 2, 1.2)), id -> id == 5);
    // #6 takes key 5; the stale #5 becomes its own episode with the first synthetic key
    assertEquals(List.of(S, 5), ids(out));
    assertEquals(5, k.keyOf(6));
    assertEquals(1, k.rebinds());
  }

  @Test
  void aFreshTrackKeepsItsEpisode() {
    var k = keys();
    k.rekey(0.0, List.of(r(1000, 5.9, 5, 0.0)), id -> true);
    // #1000 fresh (age 0.1), #1001 right next to it: two robots, no takeover.
    var out = k.rekey(0.1, List.of(r(1000, 5.9, 5, 0.0), r(1001, 6.0, 5, 0.1)), id -> id == 1000);
    assertEquals(List.of(1000, 1001), ids(out));
  }

  @Test
  void closestPairsFirst() {
    var k = keys();
    k.rekey(0.0, List.of(r(10, 0, 0, 0), r(20, 3, 0, 0)), id -> true);
    // Both lost. #30 at (0.9,0): d to 10 = 0.9, to 20 = 2.1 (too far).
    //            #31 at (0.2,0): d to 10 = 0.2, to 20 = 2.8 (too far).
    // Pairs by d: (31,10) 0.2, (30,10) 0.9 -> #31 gets 10; #30 has nothing left -> its own key.
    var out = k.rekey(0.1, List.of(r(30, 0.9, 0, 0.1), r(31, 0.2, 0, 0.1)), id -> id == 10 || id == 20);
    assertEquals(List.of(30, 10), ids(out));
  }

  @Test
  void stableTrackersNeverRebind() {
    var k = new EpisodeKeys(1.0, 0.25, false);
    k.rekey(0.0, List.of(r(0, 5.9, 5, 0.0)), id -> true);
    var out = k.rekey(0.1, List.of(r(1, 5.9, 5, 0.1)), id -> id == 0);
    assertEquals(List.of(1), ids(out)); // a new id is a different robot
    assertEquals(0, k.rebinds());
  }

  @Test
  void tracksWithoutAnIdGetStableKeysThatNeverCollideWithUnknown() {
    var k = keys();
    // Two id-less tracks (id -1, the same number as UNKNOWN): synthetic keys S and S+1.
    var a = k.rekey(0.0, List.of(r(-1, 1, 1, 0), r(-1, 5, 5, 0)), id -> false);
    assertEquals(List.of(S, S + 1), ids(a));
    assertEquals(UNKNOWN, k.aliasOf(S)); // no id to report back
    // Next loop they moved 0.3 m and 0.4 m (< 1.0): each keeps its key, matched by distance.
    var b = k.rekey(0.1, List.of(r(-1, 5.4, 5, 0.1), r(-1, 1.3, 1, 0.1)), id -> false);
    assertEquals(List.of(S + 1, S), ids(b));
    assertTrue(Set.copyOf(ids(b)).stream().allMatch(id -> id >= S));
    assertNotEquals(UNKNOWN, b.get(0).id());
  }

  @Test
  void duplicateIdInOneListIsTreatedAsANewTrack() {
    var k = keys();
    var out = k.rekey(0.0, List.of(r(7, 1, 1, 0), r(7, 4, 4, 0)), id -> false);
    assertEquals(List.of(7, S), ids(out));
    assertEquals(1.0, out.get(0).position().x(), TOL);
  }
}
