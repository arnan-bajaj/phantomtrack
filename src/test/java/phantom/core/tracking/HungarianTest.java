package phantom.core.tracking;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.Random;
import org.junit.jupiter.api.Test;

class HungarianTest {
  private static final double TOL = 1e-9;

  @Test
  void threeByThreeByHand() {
    //        c0 c1 c2
    // r0  [  4, 1, 3 ]
    // r1  [  2, 0, 5 ]
    // r2  [  3, 2, 2 ]
    // All 6 ways to give each row its own column (r0, r1, r2 -> columns):
    //   (0,1,2): 4 + 0 + 2 = 6      (0,2,1): 4 + 5 + 2 = 11
    //   (1,0,2): 1 + 2 + 2 = 5  <-- smallest
    //   (1,2,0): 1 + 5 + 3 = 9      (2,0,1): 3 + 2 + 2 = 7      (2,1,0): 3 + 0 + 3 = 6
    double[][] cost = {{4, 1, 3}, {2, 0, 5}, {3, 2, 2}};
    int[] got = Hungarian.solve(cost);
    assertArrayEquals(new int[] {1, 0, 2}, got);
    assertEquals(5.0, Hungarian.totalCost(cost, got), TOL);
  }

  @Test
  void threeByThreeWhereGreedyIsWrong() {
    //        c0 c1 c2
    // r0  [  1, 2, 9 ]
    // r1  [  1, 9, 9 ]
    // r2  [  9, 1, 2 ]
    //   (0,1,2): 1 + 9 + 2 = 12     (0,2,1): 1 + 9 + 1 = 11
    //   (1,0,2): 2 + 1 + 2 = 5  <-- smallest
    //   (1,2,0): 2 + 9 + 9 = 20     (2,0,1): 9 + 1 + 1 = 11     (2,1,0): 9 + 9 + 9 = 27
    // "Each row grabs its cheapest free column" would give r0->c0, r1->c1, r2->c2 = 12. The
    // optimum makes r0 take its second choice (2 instead of 1) so r1 can have c0.
    double[][] cost = {{1, 2, 9}, {1, 9, 9}, {9, 1, 2}};
    int[] got = Hungarian.solve(cost);
    assertArrayEquals(new int[] {1, 0, 2}, got);
    assertEquals(5.0, Hungarian.totalCost(cost, got), TOL);
  }

  @Test
  void threeRowsTwoColumnsByHand() {
    // 3 tracks, 2 detections: one track must go without.
    //        c0 c1
    // r0  [  3, 5 ]
    // r1  [  1, 4 ]
    // r2  [  2, 2 ]
    // All 6 ways to hand the two columns to two different rows (c0 -> row, c1 -> row):
    //   (r0, r1): 3 + 4 = 7      (r0, r2): 3 + 2 = 5      (r1, r0): 1 + 5 = 6
    //   (r1, r2): 1 + 2 = 3  <-- smallest
    //   (r2, r0): 2 + 5 = 7      (r2, r1): 2 + 4 = 6
    double[][] cost = {{3, 5}, {1, 4}, {2, 2}};
    int[] got = Hungarian.solve(cost);
    assertArrayEquals(new int[] {-1, 0, 1}, got); // r0 unassigned, r1 -> c0, r2 -> c1
    assertEquals(3.0, Hungarian.totalCost(cost, got), TOL);
  }

  @Test
  void twoRowsThreeColumnsByHand() {
    // 2 tracks, 3 detections: one detection is left over.
    //        c0 c1 c2
    // r0  [  3, 1, 2 ]
    // r1  [  4, 1, 5 ]
    //   (r0->c0, r1->c1): 3 + 1 = 4     (r0->c0, r1->c2): 3 + 5 = 8     (r0->c1, r1->c0): 1 + 4 = 5
    //   (r0->c1, r1->c2): 1 + 5 = 6     (r0->c2, r1->c0): 2 + 4 = 6
    //   (r0->c2, r1->c1): 2 + 1 = 3  <-- smallest (both rows want c1; r0 gives way)
    double[][] cost = {{3, 1, 2}, {4, 1, 5}};
    int[] got = Hungarian.solve(cost);
    assertArrayEquals(new int[] {2, 1}, got);
    assertEquals(3.0, Hungarian.totalCost(cost, got), TOL);
  }

  @Test
  void negativeCostsAndForbiddenPairs() {
    // Costs can be negative (the tracker's ln(det S) term is). BIG marks a forbidden pair.
    //        c0   c1
    // r0  [ -1,  BIG ]
    // r1  [ -3,  BIG ]
    //   (r0->c0, r1->c1): -1 + BIG      (r0->c1, r1->c0): BIG - 3  <-- smaller by 2
    double big = MultiTargetTracker.FORBIDDEN;
    double[][] cost = {{-1, big}, {-3, big}};
    assertArrayEquals(new int[] {1, 0}, Hungarian.solve(cost));
  }

  @Test
  void degenerateSizes() {
    assertArrayEquals(new int[0], Hungarian.solve(new double[0][0]));
    assertArrayEquals(new int[] {-1, -1}, Hungarian.solve(new double[2][0])); // no columns
    assertArrayEquals(new int[] {0}, Hungarian.solve(new double[][] {{7.5}}));
  }

  @Test
  void rejectsBadInput() {
    assertThrows(IllegalArgumentException.class,
        () -> Hungarian.solve(new double[][] {{1, Double.NaN}, {2, 3}}));
    assertThrows(IllegalArgumentException.class,
        () -> Hungarian.solve(new double[][] {{1, Double.POSITIVE_INFINITY}}));
    assertThrows(IllegalArgumentException.class,
        () -> Hungarian.solve(new double[][] {{1, 2}, {3}})); // ragged
  }

  @Test
  void matchesBruteForceOnRandomMatrices() {
    // Not hand-computed: checks optimality against trying every assignment, 2000 random matrices
    // up to 5 x 6 (both tall and wide), costs in [-10, 10].
    Random rng = new Random(2027);
    for (int trial = 0; trial < 2000; trial++) {
      int rows = 1 + rng.nextInt(5);
      int cols = 1 + rng.nextInt(6);
      double[][] cost = new double[rows][cols];
      for (double[] row : cost) {
        for (int c = 0; c < cols; c++) {
          row[c] = Math.round((rng.nextDouble() * 20 - 10) * 10) / 10.0;
        }
      }
      int[] got = Hungarian.solve(cost);
      // valid: min(rows, cols) rows assigned, no column used twice
      boolean[] used = new boolean[cols];
      int assigned = 0;
      for (int c : got) {
        if (c >= 0) {
          assertEquals(false, used[c], "column used twice");
          used[c] = true;
          assigned++;
        }
      }
      assertEquals(Math.min(rows, cols), assigned);
      assertEquals(bruteForce(cost, 0, new boolean[cols], rows - Math.min(rows, cols)),
          Hungarian.totalCost(cost, got), 1e-9, "trial " + trial);
    }
  }

  /** Cheapest way to finish from this row on, allowed to skip {@code skipsLeft} more rows. */
  private static double bruteForce(double[][] cost, int row, boolean[] used, int skipsLeft) {
    if (row == cost.length) {
      return skipsLeft == 0 ? 0.0 : Double.POSITIVE_INFINITY;
    }
    double best = Double.POSITIVE_INFINITY;
    if (skipsLeft > 0) {
      best = bruteForce(cost, row + 1, used, skipsLeft - 1);
    }
    for (int c = 0; c < used.length; c++) {
      if (!used[c]) {
        used[c] = true;
        best = Math.min(best, cost[row][c] + bruteForce(cost, row + 1, used, skipsLeft));
        used[c] = false;
      }
    }
    return best;
  }
}
