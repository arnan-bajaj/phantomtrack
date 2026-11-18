package phantom.core.tracking;

/**
 * M-of-N confirmation: a new track only becomes real after the camera has seen it M times within
 * its first N frames. One stray false detection therefore never becomes a robot.
 *
 * <pre>
 *   frame 1 is the detection that started the track (a hit)
 *   every later camera frame is a hit (a detection was matched to the track) or a miss
 *
 *   CONFIRMED  as soon as  hits   >= M          (this always happens within the first N frames)
 *   REJECTED   as soon as  misses >  N - M      (M hits in the first N frames is now impossible)
 *   PENDING    otherwise
 *
 *   Because hits + misses = frames, by frame N one of the two has happened:
 *   hits < M  means  misses = N - hits > N - M.
 * </pre>
 *
 * Example with the default 3 of 5 (H = hit, M = miss): H H H confirms at frame 3; H M H M H
 * confirms at frame 5; H M M M is rejected at frame 4 (3 misses > 5 - 3 = 2).
 */
public final class MOfN {
  /** Where a track is in its trial period. */
  public enum Decision {
    PENDING,
    CONFIRMED,
    REJECTED
  }

  private final int m;
  private final int n;
  private int hits = 0;
  private int misses = 0;
  private Decision decision = Decision.PENDING;

  /**
   * @param m hits needed
   * @param n frames allowed (counting the frame that started the track), n >= m >= 1
   */
  public MOfN(int m, int n) {
    if (m < 1 || n < m) {
      throw new IllegalArgumentException("need 1 <= M <= N, got " + m + " of " + n);
    }
    this.m = m;
    this.n = n;
  }

  /** Record one frame. Once CONFIRMED or REJECTED the answer never changes. */
  public Decision add(boolean hit) {
    if (decision != Decision.PENDING) {
      return decision;
    }
    if (hit) {
      hits++;
    } else {
      misses++;
    }
    if (hits >= m) {
      decision = Decision.CONFIRMED;
    } else if (misses > n - m) {
      decision = Decision.REJECTED;
    }
    return decision;
  }

  public Decision decision() {
    return decision;
  }

  public int hits() {
    return hits;
  }

  public int misses() {
    return misses;
  }
}
