package phantom.core.manager;

import java.util.List;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Picks which opponent to guard when the driver leaves focus on "auto". Stays on its scorer until
 * that opponent stops being a threat (it turns away from the goal, e.g. after scoring), then picks
 * up the next one coming in. Jumps early only if someone else is much closer to scoring.
 *
 * <pre>
 *   closing = v_opp . unit(goal - p_opp)            (m/s toward the goal)
 *   T       = |goal - p_opp| / closing   if closing >= MIN_CLOSING   (time to goal, s)
 *           = infinity                    otherwise                   (not coming at the goal)
 *
 *   current lost (untracked)                          -> take the lowest finite T now
 *   current T infinite for dwell seconds in a row     -> take the lowest finite T (if any)
 *   some c with T_c < T_current - margin, dwell in a row -> switch to c
 *   otherwise                                         -> keep current
 * </pre>
 */
public final class TargetSelector {
  public static final double MIN_CLOSING = 0.5; // m/s

  private final double marginSec;
  private final double dwellSec;
  private int current = -1;
  private int challenger = -1;
  private double challengerSince = Double.NaN;
  private double idleSince = Double.NaN;
  private int switches = 0;

  /**
   * @param marginSec how much sooner (s) another opponent must reach the goal to switch early
   * @param dwellSec how long (s) a reason to switch must hold before switching
   */
  public TargetSelector(double marginSec, double dwellSec) {
    this.marginSec = marginSec;
    this.dwellSec = dwellSec;
  }

  public static TargetSelector defaults() {
    return new TargetSelector(2.0, 0.5);
  }

  /** Time for this opponent to reach the goal (s), or infinity if it isn't heading there. */
  public static double timeToGoal(TrackedRobot r, double nowSec, Vec2 goal) {
    Vec2 toGoal = goal.minus(r.positionAt(nowSec));
    double closing = r.velocity().dot(toGoal.normalized());
    return closing >= MIN_CLOSING ? toGoal.norm() / closing : Double.POSITIVE_INFINITY;
  }

  /** The opponent id to guard, or -1 if nobody is tracked. Call once per loop. */
  public int select(double nowSec, List<TrackedRobot> opponents, Vec2 goal) {
    if (opponents.isEmpty()) {
      return current; // keep the choice through brief tracking gaps; modes search for it
    }
    TrackedRobot best = null;
    double bestT = Double.POSITIVE_INFINITY;
    TrackedRobot nearestToGoal = null;
    double nearestDist = Double.POSITIVE_INFINITY;
    double currentT = Double.NaN;
    for (TrackedRobot r : opponents) {
      double t = timeToGoal(r, nowSec, goal);
      if (t < bestT) {
        bestT = t;
        best = r;
      }
      double d = r.positionAt(nowSec).distanceTo(goal);
      if (d < nearestDist) {
        nearestDist = d;
        nearestToGoal = r;
      }
      if (r.id() == current) {
        currentT = t;
      }
    }

    // First pick, or our target is no longer tracked: take the biggest threat now.
    if (current < 0 || Double.isNaN(currentT)) {
      TrackedRobot pick = best != null ? best : nearestToGoal;
      if (current >= 0 && pick.id() != current) {
        switches++;
      }
      current = pick.id();
      resetTimers();
      return current;
    }

    // Our target stopped coming at the goal: after dwell, move to the next threat.
    if (Double.isInfinite(currentT)) {
      if (Double.isNaN(idleSince)) {
        idleSince = nowSec;
      }
      if (best != null && best.id() != current && nowSec - idleSince >= dwellSec - 1e-9) {
        current = best.id();
        switches++;
        resetTimers();
      }
      return current;
    }
    idleSince = Double.NaN;

    // Someone else is much closer to scoring: after dwell, switch early.
    if (best != null && best.id() != current && bestT < currentT - marginSec) {
      if (challenger != best.id()) {
        challenger = best.id();
        challengerSince = nowSec;
      } else if (nowSec - challengerSince >= dwellSec - 1e-9) {
        current = challenger;
        switches++;
        resetTimers();
      }
    } else {
      challenger = -1;
    }
    return current;
  }

  private void resetTimers() {
    challenger = -1;
    challengerSince = Double.NaN;
    idleSince = Double.NaN;
  }

  public int current() {
    return current;
  }

  /** Number of times the guarded opponent changed (not counting the first pick). */
  public int switches() {
    return switches;
  }
}
