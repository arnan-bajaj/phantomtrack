package phantom.core.rules;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.IntPredicate;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Gives every opponent the pin guard sees a stable EPISODE KEY, so pin time and contact belief
 * belong to the contact episode, not to a track id that an anonymous tracker may change.
 *
 * <p>A tracker's own re-acquisition (MultiTargetTracker's handoff by proximity or elimination)
 * already hands the lost robot's id to its new track; then the id, and so the key, simply stays
 * the same. This class covers the switches the tracker did not catch: the robot we are pushing
 * loses its track and comes back under a new id, or a tracker reports tracks with no id at all
 * (negative ids).
 *
 * <pre>
 *   Per loop, for the reported tracks r (position p_r = r.positionAt(now), age_r = now - t_r):
 *
 *   1. KNOWN id (seen before, id >= 0):  key(r) = the key it had.
 *   (A track with id < 0, or with an id already used earlier in the same list, has NO id here.)
 *
 *   2. NEW track (id never seen, or no id) may take over a LOST episode E (only for trackers
 *      whose ids can change, see {@code rebindNewIds}; with stable ids a new id is a new robot):
 *        E has an open episode (episodeOpen(E)), E != UNKNOWN,
 *        E's current track is not reported this loop, or is reported but stale (age > freshSec),
 *        d = |p_r - lastPos(E)| <= rebindRadius
 *          lastPos(E) = E's position at its last fresh sighting (age <= freshSec); before any
 *                       fresh sighting, its last reported position
 *      Pairs are taken closest d first (ties: smaller key, then smaller id), each track and each
 *      episode at most once per loop. A taken-over episode's old track, if it is still reported,
 *      gets a brand-new key (it is a separate, stale track now).
 *   3. Otherwise a new key:  key = id  if id >= 0 and no live key equals it,
 *                            else the next synthetic key (from SYNTHETIC_START up).
 *      (A key is live while a track is behind it or its episode is open; then it is forgotten.)
 *      Tracks with no id (id < 0) also try to keep last loop's key first: the closest key that
 *      was given to an id-less track last loop, within rebindRadius.
 *
 *   The keyed view is the input list, same order, with id replaced by the key; positions,
 *   velocities and timestamps are unchanged. aliasOf(key) is the track id now behind a key
 *   (UNKNOWN for an id-less track, or if no track is behind it this loop).
 * </pre>
 *
 * With stable ids (the perfect tracker, known-id Kalman tracking) every key equals its id, so the
 * guard behaves exactly as if it were keyed by id.
 *
 * <p>Deterministic, no clock reads (time is passed in). Keeps state between calls, so use one
 * instance per guard and call {@link #rekey} once per loop.
 */
public final class EpisodeKeys {
  /** Synthetic keys (for reused or missing ids) count up from here, far above any tracker id. */
  public static final int SYNTHETIC_START = 1 << 30;

  private static final int UNKNOWN = ContactDetector.UNKNOWN;

  private final double rebindRadius;
  private final double freshSec;
  private boolean rebindNewIds;
  private final Map<Integer, Integer> keyOfId = new HashMap<>();
  private final Map<Integer, Vec2> lastPos = new HashMap<>();
  private final Set<Integer> seenFresh = new HashSet<>(); // keys with at least one fresh sighting
  private final Map<Integer, Integer> aliasNow = new HashMap<>();
  private Map<Integer, Vec2> idlessLastLoop = new HashMap<>(); // key -> position
  private int nextSynthetic = SYNTHETIC_START;
  private int rebinds = 0;

  /**
   * @param rebindRadius how close a new track must be to where a lost episode's opponent was (m)
   * @param freshSec a track older than this cannot hold on to its episode against a new track (s)
   * @param rebindNewIds false skips step 2 (for trackers whose ids never change: a new id there
   *     is a different robot)
   */
  public EpisodeKeys(double rebindRadius, double freshSec, boolean rebindNewIds) {
    this.rebindRadius = rebindRadius;
    this.freshSec = freshSec;
    this.rebindNewIds = rebindNewIds;
  }

  /** Turn step 2 on (trackers that may change ids) or off (stable ids). */
  public void setRebindNewIds(boolean on) {
    this.rebindNewIds = on;
  }

  /**
   * Re-label this loop's tracks with episode keys (see the class comment).
   *
   * @param episodeOpen true for keys that have an open pin episode
   */
  public List<TrackedRobot> rekey(double nowSec, List<TrackedRobot> tracks, IntPredicate episodeOpen) {
    int n = tracks.size();
    int[] keys = new int[n];
    boolean[] isNew = new boolean[n];
    int[] ids = new int[n]; // usable id, or -1: a repeated id in one list counts as no id
    Set<Integer> seenIds = new HashSet<>();
    for (int i = 0; i < n; i++) {
      int id = tracks.get(i).id();
      ids[i] = id >= 0 && seenIds.add(id) ? id : -1;
    }
    Map<Integer, Integer> trackOfKey = new HashMap<>(); // key -> index into tracks

    // 1. Known ids keep their key.
    for (int i = 0; i < n; i++) {
      int id = ids[i];
      Integer k = id >= 0 ? keyOfId.get(id) : null;
      if (k == null || trackOfKey.containsKey(k)) {
        isNew[i] = true;
      } else {
        keys[i] = k;
        trackOfKey.put(k, i);
      }
    }

    // 2. New tracks may take over lost episodes, closest first.
    record Pair(int track, int key, double d) {}
    List<Pair> pairs = new ArrayList<>();
    for (Map.Entry<Integer, Vec2> e : lastPos.entrySet()) {
      int key = e.getKey();
      if (!rebindNewIds || key == UNKNOWN || !episodeOpen.test(key)) {
        continue;
      }
      Integer holder = trackOfKey.get(key);
      boolean lost = holder == null || nowSec - tracks.get(holder).timestampSec() > freshSec;
      if (!lost) {
        continue;
      }
      for (int i = 0; i < n; i++) {
        if (!isNew[i]) {
          continue;
        }
        double d = tracks.get(i).positionAt(nowSec).distanceTo(e.getValue());
        if (d <= rebindRadius) {
          pairs.add(new Pair(i, key, d));
        }
      }
    }
    pairs.sort(
        Comparator.comparingDouble(Pair::d)
            .thenComparingInt(Pair::key)
            .thenComparingInt(p -> ids[p.track()]));
    Set<Integer> takenKeys = new HashSet<>();
    for (Pair p : pairs) {
      if (!isNew[p.track()] || takenKeys.contains(p.key())) {
        continue;
      }
      Integer oldHolder = trackOfKey.get(p.key());
      if (oldHolder != null) { // a stale track of the same robot: it becomes its own episode
        int fresh = newKey(ids[oldHolder], trackOfKey);
        keys[oldHolder] = fresh;
        trackOfKey.put(fresh, oldHolder);
        bind(ids[oldHolder], fresh);
      }
      keys[p.track()] = p.key();
      trackOfKey.put(p.key(), p.track());
      bind(ids[p.track()], p.key());
      isNew[p.track()] = false;
      takenKeys.add(p.key());
      rebinds++;
    }

    // 3. Everything else: an id-less track keeps last loop's key if it can, otherwise a new key.
    for (int i = 0; i < n; i++) {
      if (!isNew[i]) {
        continue;
      }
      TrackedRobot r = tracks.get(i);
      Integer k = null;
      if (ids[i] < 0) {
        double best = Double.POSITIVE_INFINITY;
        for (int key : new TreeSet<>(idlessLastLoop.keySet())) { // ties: smaller key
          double d = r.positionAt(nowSec).distanceTo(idlessLastLoop.get(key));
          if (!trackOfKey.containsKey(key) && d <= rebindRadius && d < best) {
            best = d;
            k = key;
          }
        }
      }
      if (k == null) {
        k = newKey(ids[i], trackOfKey);
      }
      keys[i] = k;
      trackOfKey.put(k, i);
      bind(ids[i], k);
    }

    // Bookkeeping: positions, aliases, the keyed view.
    aliasNow.clear();
    Map<Integer, Vec2> idless = new HashMap<>();
    List<TrackedRobot> out = new ArrayList<>(n);
    for (int i = 0; i < n; i++) {
      TrackedRobot r = tracks.get(i);
      int k = keys[i];
      Vec2 p = r.positionAt(nowSec);
      if (nowSec - r.timestampSec() <= freshSec) {
        lastPos.put(k, p);
        seenFresh.add(k);
      } else if (!seenFresh.contains(k)) {
        lastPos.put(k, p); // no fresh sighting yet: the last reported position
      }
      aliasNow.put(k, ids[i] >= 0 ? ids[i] : UNKNOWN);
      if (ids[i] < 0) {
        idless.put(k, p);
      }
      out.add(new TrackedRobot(k, r.position(), r.velocity(), r.timestampSec(), r.confidence()));
    }
    idlessLastLoop = idless;
    // Forget keys nobody can come back to: no open episode and no track behind them now.
    lastPos.keySet().removeIf(k -> !episodeOpen.test(k) && !aliasNow.containsKey(k));
    keyOfId.values().removeIf(k -> !lastPos.containsKey(k));
    seenFresh.retainAll(lastPos.keySet());
    return out;
  }

  private void bind(int id, int key) {
    if (id >= 0) {
      keyOfId.put(id, key);
    }
  }

  /** The id itself if that key is free (not remembered, not given out this loop), else synthetic. */
  private int newKey(int id, Map<Integer, Integer> takenThisLoop) {
    if (id >= 0 && !lastPos.containsKey(id) && !takenThisLoop.containsKey(id)) {
      return id;
    }
    return nextSynthetic++;
  }

  /** Episode key of a track id (UNKNOWN maps to itself); -2 if the id is not known. */
  public int keyOf(int trackId) {
    if (trackId == UNKNOWN) {
      return UNKNOWN;
    }
    return keyOfId.getOrDefault(trackId, -2);
  }

  /**
   * Track id behind this key on the last {@link #rekey}: UNKNOWN for UNKNOWN, for an id-less
   * track, or when no reported track is behind the key right now.
   */
  public int aliasOf(int key) {
    return aliasNow.getOrDefault(key, UNKNOWN);
  }

  /** Times a new track took over a lost episode (step 2). */
  public int rebinds() {
    return rebinds;
  }
}
