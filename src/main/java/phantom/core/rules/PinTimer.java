package phantom.core.rules;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
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
 *
 * <p>The referee in the sim uses the geometric {@code update} methods with true positions. The
 * robot's own guard cannot see true positions, so it uses {@link #updateContacts}: contact comes
 * from {@link ContactDetector} and the caller decides when an opponent has separated.
 */
public final class PinTimer {
  public static final double CONTACT_TOLERANCE = 0.05;

  private final PinRules rules;
  private final double robotHalfSize;
  private final Map<Integer, Double> contactSeconds = new HashMap<>();
  private final Map<Integer, Boolean> touching = new HashMap<>();
  private final Map<Integer, Double> lastContact = new HashMap<>();
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
    updateContacts(
        nowSec,
        opponents.keySet(),
        id ->
            (gap(self, opponents.get(id)) <= CONTACT_TOLERANCE || forcedContact.test(id))
                && pressing.test(id),
        id -> gap(self, opponents.get(id)) >= rules.resetSeparation());
  }

  /**
   * General form: the caller says who we are in contact with and who has separated.
   *
   * <pre>
   *   for each id:
   *     in contact:      new episode at 0 s if none is open; otherwise + dt if also in contact
   *                      on the previous call (the first loop of a touch adds nothing)
   *     not in contact:  episode cleared if separated(id), otherwise paused
   * </pre>
   *
   * @param ids every id to update this loop (opponents, plus any stand-in id such as {@link
   *     ContactDetector#UNKNOWN})
   * @param inContact true for ids we are touching and pressing right now
   * @param separated true for ids whose pin episode is over (only consulted when not in contact)
   */
  public void updateContacts(
      double nowSec, Collection<Integer> ids, IntPredicate inContact, IntPredicate separated) {
    double dt = Double.isNaN(lastTime) ? 0.0 : Math.max(0.0, nowSec - lastTime);
    lastTime = nowSec;
    for (int id : ids) {
      boolean contact = inContact.test(id);
      boolean wasTouching = touching.getOrDefault(id, false);
      touching.put(id, contact);
      if (contact) {
        if (!contactSeconds.containsKey(id)) {
          contactSeconds.put(id, 0.0); // new episode
        } else if (wasTouching) {
          contactSeconds.put(id, contactSeconds.get(id) + dt); // only time spent touching counts
        }
        lastContact.put(id, nowSec);
      } else if (separated.test(id)) {
        contactSeconds.remove(id);
        lastContact.remove(id);
      }
    }
  }

  /**
   * Makes this id's contact time at least {@code seconds}, opening an episode if needed. Used when
   * contact we were timing under one id turns out to belong to another (e.g. an unidentified
   * blocker comes into camera view). Does nothing for seconds <= 0.
   */
  public void raiseTo(int id, double seconds) {
    if (seconds > 0.0) {
      contactSeconds.merge(id, seconds, Math::max);
    }
  }

  /** Time of the last contact in this id's open episode (s); NaN if there is none. */
  public double lastContactSec(int id) {
    return lastContact.getOrDefault(id, Double.NaN);
  }

  /** Ids with an open episode, in ascending order. */
  public Set<Integer> openEpisodes() {
    return new TreeSet<>(contactSeconds.keySet());
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
