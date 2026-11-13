package phantom.core.rules;

import java.util.HashMap;
import java.util.Map;
import java.util.function.IntPredicate;
import phantom.core.geometry.Vec2;

/**
 * Tracks pin "episodes" between our robot and each opponent.
 *
 * <pre>
 *   gap        = |p_opp - p_self| - 2 * robotHalfSize        (bumper-to-bumper, robots as circles)
 *   in contact = (gap <= CONTACT_TOLERANCE  or  our own sensors say we're blocked by them)
 *                and  we are pressing into that opponent
 *                (callers that can't tell, like RulesGuard, assume we always are: conservative)
 *   episode starts on first contact; contact time accumulates only while in contact;
 *   episode ends (timer cleared) once gap >= resetSeparation.
 * </pre>
 *
 * Brief separations shorter than the reset distance pause the count instead of clearing it, so
 * "bumping" an opponent repeatedly still adds up. That is deliberately conservative.
 */
public final class PinTimer {
  public static final double CONTACT_TOLERANCE = 0.05;

  private final PinRules rules;
  private final double robotHalfSize;
  private final Map<Integer, Double> contactSeconds = new HashMap<>();
  private final Map<Integer, Boolean> touching = new HashMap<>();
  private double lastTime = Double.NaN;

  public PinTimer(PinRules rules, double robotHalfSize) {
    this.rules = rules;
    this.robotHalfSize = robotHalfSize;
  }

  /** Call once per loop with every opponent's current position. Assumes we press into everyone. */
  public void update(double nowSec, Vec2 self, Map<Integer, Vec2> opponents) {
    update(nowSec, self, opponents, id -> true);
  }

  /**
   * Call once per loop with every opponent's current position.
   *
   * @param pressing true for opponents we are actively driving into; contact with anyone else
   *     (e.g. while we back away, or when they push us) does not add pin time
   */
  public void update(double nowSec, Vec2 self, Map<Integer, Vec2> opponents, IntPredicate pressing) {
    update(nowSec, self, opponents, pressing, id -> false);
  }

  /**
   * Full version.
   *
   * @param forcedContact true for opponents we know we are touching from our own sensors (e.g. we
   *     are pushing and not moving), even if their estimated position says otherwise
   */
  public void update(
      double nowSec,
      Vec2 self,
      Map<Integer, Vec2> opponents,
      IntPredicate pressing,
      IntPredicate forcedContact) {
    double dt = Double.isNaN(lastTime) ? 0.0 : Math.max(0.0, nowSec - lastTime);
    lastTime = nowSec;
    for (Map.Entry<Integer, Vec2> e : opponents.entrySet()) {
      int id = e.getKey();
      double gap = gap(self, e.getValue());
      boolean inContact =
          (gap <= CONTACT_TOLERANCE || forcedContact.test(id)) && pressing.test(id);
      boolean wasTouching = touching.getOrDefault(id, false);
      touching.put(id, inContact);
      if (inContact) {
        if (!contactSeconds.containsKey(id)) {
          contactSeconds.put(id, 0.0); // new episode
        } else if (wasTouching) {
          contactSeconds.put(id, contactSeconds.get(id) + dt); // only time spent touching counts
        }
      } else if (gap >= rules.resetSeparation()) {
        contactSeconds.remove(id);
      }
    }
  }

  /** Bumper-to-bumper gap between two robots (m); negative if overlapping. */
  public double gap(Vec2 self, Vec2 opponent) {
    return opponent.distanceTo(self) - 2.0 * robotHalfSize;
  }

  /** Contact time accumulated in the current episode with this opponent (s); 0 if none. */
  public double contactSeconds(int id) {
    return contactSeconds.getOrDefault(id, 0.0);
  }

  /** True if an episode with this opponent is open (not yet separated by resetSeparation). */
  public boolean episodeOpen(int id) {
    return contactSeconds.containsKey(id);
  }

  public boolean touching(int id) {
    return touching.getOrDefault(id, false);
  }

  public PinRules rules() {
    return rules;
  }
}
