package phantom.core.manager;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import phantom.core.geometry.Vec2;
import phantom.core.modes.HoldGroundMode;
import phantom.core.tracking.TrackedRobot;

/**
 * Decides, per tracked opponent, whether hold ground may push on it (audit U1): hold pushes only
 * opponents closing on us or on the goal. The closing speed is {@link
 * HoldGroundMode#closingSpeed}:
 *
 * <pre>
 *   n       = unit(p_opp - p_self),   g = unit(goal - p_opp)
 *   closing = max(-v_opp . n, v_opp . g)       (m/s; > 0 = coming at us or at the goal)
 * </pre>
 *
 * With hysteresis, so camera velocity noise cannot make hold flicker:
 *
 * <pre>
 *   hold allowed -> released  when closing < OFF_BELOW = -0.25 m/s on every update for
 *                             OFF_SECONDS = 0.2 s  (t - t_first_below >= 0.2)
 *   released -> hold allowed  as soon as closing > ON_ABOVE = -0.10 m/s
 * </pre>
 *
 * Why these numbers:
 *
 * <ul>
 *   <li><b>Off below -0.25 m/s, not 0.</b> In a full pushing match both robots stall and the
 *       closing speed is 0. That is exactly when hold must keep pushing, so the off threshold sits
 *       below 0: only an opponent clearly driving away loses the hold.
 *   <li><b>For 0.2 s (10 loops at 20 ms).</b> A single noisy velocity under -0.25 m/s is not
 *       enough; it has to stay there for 10 updates in a row. An opponent really leaving at only
 *       0.5 m/s moves 0.1 m in that time, so the delay costs little.
 *   <li><b>Back on above -0.10 m/s, at once.</b> The 0.15 m/s band between the two thresholds
 *       means noise around -0.25 cannot switch it back and forth; no delay on the way back,
 *       because an opponent that turns around is a threat right away.
 * </ul>
 *
 * A new track id starts with hold allowed (as before this rule). State for ids that are no longer
 * tracked is dropped. This is state with a timer, so it lives here in the manager, not in the
 * mode: {@link HoldGroundMode} stays a pure function and only reads the released ids from the
 * {@link phantom.core.modes.DefenseContext}.
 */
public final class HoldLatch {
  public static final double OFF_BELOW = -0.25;
  public static final double OFF_SECONDS = 0.2;
  public static final double ON_ABOVE = -0.10;
  private static final double EPS = 1e-9; // 0.2 s of 20 ms steps in floating point

  private static final class Entry {
    boolean released;
    double belowSince = Double.NaN;
  }

  private final Map<Integer, Entry> entries = new HashMap<>();
  private int switches = 0;

  /**
   * Updates every tracked opponent and returns the ids hold must not push on.
   *
   * @param nowSec current time (s)
   * @param self our position
   * @param opponents current tracks
   * @param goal the goal we defend
   */
  public Set<Integer> update(double nowSec, Vec2 self, List<TrackedRobot> opponents, Vec2 goal) {
    Set<Integer> seen = new HashSet<>();
    Set<Integer> released = new HashSet<>();
    for (TrackedRobot r : opponents) {
      seen.add(r.id());
      Entry e = entries.computeIfAbsent(r.id(), k -> new Entry());
      double c = HoldGroundMode.closingSpeed(self, r.positionAt(nowSec), r.velocity(), goal);
      if (!e.released) {
        if (c < OFF_BELOW) {
          if (Double.isNaN(e.belowSince)) {
            e.belowSince = nowSec;
          }
          if (nowSec - e.belowSince >= OFF_SECONDS - EPS) {
            e.released = true;
            switches++;
          }
        } else {
          e.belowSince = Double.NaN;
        }
      } else if (c > ON_ABOVE) {
        e.released = false;
        e.belowSince = Double.NaN;
        switches++;
      }
      if (e.released) {
        released.add(r.id());
      }
    }
    entries.keySet().retainAll(seen);
    return released;
  }

  /** How many times any opponent's hold permission flipped (either way) so far. */
  public int switches() {
    return switches;
  }
}
