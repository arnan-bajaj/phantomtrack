package phantom.core.tracking;

import java.util.Arrays;

/**
 * Optimal one-to-one assignment (the "Hungarian" / Kuhn-Munkres method), written from scratch so
 * phantom.core keeps zero dependencies.
 *
 * <p>Problem: given {@code cost[r][c]} for R rows (e.g. tracks) and C columns (e.g. detections),
 * pair up min(R, C) of them, each row and each column used at most once, so that the sum of the
 * chosen costs is as small as possible. Rectangular matrices are fine: with more rows than columns
 * some rows stay unassigned (-1); with more columns than rows some columns are simply not used.
 *
 * <pre>
 * Shortest-augmenting-path form with potentials (needs n rows <= m columns; otherwise the
 * matrix is transposed first):
 *
 *   keep a potential u[i] per row and v[j] per column so that the "reduced cost"
 *       cost[i][j] - u[i] - v[j]  >= 0  for every pair,  and  = 0  for every matched pair.
 *   add the rows one at a time. For the new row i:
 *     grow a tree from i along zero-reduced-cost pairs (like Dijkstra):
 *       delta = smallest reduced cost from a row in the tree to a column outside it
 *       u += delta on the tree's rows,  v -= delta on the tree's columns
 *       (nothing goes negative, matched pairs stay at 0, and one more column joins the tree)
 *     stop when the tree reaches a column nobody uses, then flip the pairs along that path.
 *
 * Each row costs O(n m), so the whole thing is O(n^2 m), i.e. O(n^3) for a square matrix.
 * </pre>
 *
 * <p>Costs may be negative but must be finite. To forbid a pair, give it a cost far above every
 * real cost (see {@link MultiTargetTracker#FORBIDDEN}) and drop it from the answer afterwards.
 */
public final class Hungarian {
  private Hungarian() {}

  /**
   * @param cost cost[row][column]; every row the same length; finite values only
   * @return for each row, the column assigned to it, or -1 if the row is left out (this only
   *     happens when there are more rows than columns)
   * @throws IllegalArgumentException if the matrix is ragged or contains NaN / infinity
   */
  public static int[] solve(double[][] cost) {
    int rows = cost.length;
    int[] columnOfRow = new int[rows];
    Arrays.fill(columnOfRow, -1);
    if (rows == 0) {
      return columnOfRow;
    }
    int cols = cost[0].length;
    for (double[] row : cost) {
      if (row.length != cols) {
        throw new IllegalArgumentException("cost matrix rows must all have the same length");
      }
      for (double c : row) {
        if (!Double.isFinite(c)) {
          throw new IllegalArgumentException("costs must be finite, got " + c);
        }
      }
    }
    if (cols == 0) {
      return columnOfRow;
    }
    if (rows <= cols) {
      int[] rowOfColumn = solveWide(cost, rows, cols, false);
      for (int j = 0; j < cols; j++) {
        if (rowOfColumn[j] >= 0) {
          columnOfRow[rowOfColumn[j]] = j;
        }
      }
    } else {
      // More rows than columns: solve the transposed problem (its "rows" are our columns).
      int[] columnOfOriginalRow = solveWide(cost, cols, rows, true);
      for (int r = 0; r < rows; r++) {
        columnOfRow[r] = columnOfOriginalRow[r];
      }
    }
    return columnOfRow;
  }

  /** Sum of the costs of the assigned pairs (rows mapped to -1 add nothing). */
  public static double totalCost(double[][] cost, int[] columnOfRow) {
    double total = 0.0;
    for (int r = 0; r < columnOfRow.length; r++) {
      if (columnOfRow[r] >= 0) {
        total += cost[r][columnOfRow[r]];
      }
    }
    return total;
  }

  /**
   * Core algorithm for n <= m. Element (i, j) of the matrix being solved is {@code cost[i][j]},
   * or {@code cost[j][i]} when {@code transposed}.
   *
   * @return for each of the m columns, the row matched to it, or -1 if the column is unused
   */
  private static int[] solveWide(double[][] cost, int n, int m, boolean transposed) {
    // Index 0 is a dummy column that holds the row being added; real rows/columns are 1-based.
    double[] u = new double[n + 1];
    double[] v = new double[m + 1];
    int[] rowOf = new int[m + 1]; // rowOf[j] = row matched to column j, 0 = free
    int[] previous = new int[m + 1]; // previous column on the path that reached column j
    double[] minReduced = new double[m + 1];
    boolean[] inTree = new boolean[m + 1];

    for (int i = 1; i <= n; i++) {
      rowOf[0] = i;
      int j0 = 0;
      Arrays.fill(minReduced, Double.POSITIVE_INFINITY);
      Arrays.fill(inTree, false);
      do {
        inTree[j0] = true;
        int i0 = rowOf[j0];
        double delta = Double.POSITIVE_INFINITY;
        int j1 = -1;
        for (int j = 1; j <= m; j++) {
          if (inTree[j]) {
            continue;
          }
          double a = transposed ? cost[j - 1][i0 - 1] : cost[i0 - 1][j - 1];
          double reduced = a - u[i0] - v[j];
          if (reduced < minReduced[j]) {
            minReduced[j] = reduced;
            previous[j] = j0;
          }
          if (minReduced[j] < delta) {
            delta = minReduced[j];
            j1 = j;
          }
        }
        for (int j = 0; j <= m; j++) {
          if (inTree[j]) {
            u[rowOf[j]] += delta;
            v[j] -= delta;
          } else {
            minReduced[j] -= delta;
          }
        }
        j0 = j1;
      } while (rowOf[j0] != 0);
      // Flip the matching along the path back to the dummy column.
      do {
        int j1 = previous[j0];
        rowOf[j0] = rowOf[j1];
        j0 = j1;
      } while (j0 != 0);
    }

    int[] result = new int[m];
    for (int j = 1; j <= m; j++) {
      result[j - 1] = rowOf[j] - 1;
    }
    return result;
  }
}
