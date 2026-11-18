package phantom.core.tracking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import phantom.core.geometry.Vec2;

/**
 * Opponent tracker for ANONYMOUS camera detections. A real camera does not say which robot a
 * sighting belongs to: it reports unlabeled positions of opponents AND alliance partners, with a
 * bumper-color guess that is sometimes wrong, plus the odd false positive. This class turns that
 * into the same smooth, id-carrying opponent tracks that {@link KalmanOpponentTracker} produces
 * when identity is handed to it for free.
 *
 * <pre>
 * Per camera frame (capture time t, detections z_1..z_D, each with a color label):
 *
 *  1. Predict every track i to time t, without committing ({@link AxisKalman} peek methods):
 *       dt_i = t - t_i          p_i = position_i + velocity_i * dt_i
 *       S_i  = diag(P00x_i(dt_i) + R, P00y_i(dt_i) + R)          R = measurementStd^2
 *  2. Gate, 2 degrees of freedom (see {@link Gating}):
 *       d2_ij = (z_j - p_i)' S_i^-1 (z_j - p_i)  must be  <= 9.210
 *  3. Cost of giving detection j to track i = -2 ln(likelihood), constants dropped:
 *       c_ij = d2_ij + ln(det S_i) - 2 ln P(label_j | color belief of track i)    if gated in
 *            = FORBIDDEN                                                           otherwise
 *     ln(det S_i) = ln(Sx_i * Sy_i). It keeps a track that has coasted for a while (huge S, so
 *     every d2 looks small) from stealing a detection from the track that predicted it sharply.
 *     The label term makes an "opponent" track reluctant to take a "partner" detection.
 *  4. Global nearest neighbor: {@link Hungarian} picks the pairing with the most gated pairs
 *     and, among those, the smallest total cost. One detection per track, one track per detection.
 *  5. Matched track: Kalman predict(dt_i) + update(z_j) per axis, color belief update, a hit.
 *     Unmatched track: a miss; its state is left alone (it coasts on its old estimate).
 *     Unmatched detection: starts a TENTATIVE track at z_j, velocity 0 +- 2 m/s.
 *
 * Life cycle (see {@link MOfN}, {@link Stage}):
 *     TENTATIVE --M hits within its first N frames (default 3 of 5)--> CONFIRMED
 *     TENTATIVE --more than N - M misses, or no hit for coastTimeoutSec--> deleted
 *     CONFIRMED --missed the latest frame--> COASTING --hit--> CONFIRMED
 *     CONFIRMED / COASTING --no hit for coastTimeoutSec--> deleted
 *
 * Reported (what getOpponents returns): CONFIRMED or COASTING tracks with P(opponent) > 0.5
 * (see {@link ColorBelief}). Each is the track's last filtered state, stamped with the capture
 * time of the detection that produced it.
 *     confidence = P(opponent) * sigma0^2 / (sigma0^2 + var_now)
 *       var_now  = position variance predicted from that state to "now" (mean of x and y)
 *       sigma0   = confidenceSigma: the uncertainty at which confidence has halved
 *
 * Latency and out-of-order frames (same behavior as KalmanOpponentTracker):
 *     Every detection is applied at its capture time and the output carries that timestamp, so
 *     the modes extrapolate to the present with positionAt(now). Frames are processed in capture
 *     order; a frame that is not newer than the last processed one is dropped whole.
 *
 * Track ids and re-acquisition ("handoff"):
 *     A track gets a new id when it is created and keeps it for as long as it is reported.
 *     When a reported id loses its track (timed out, or its color belief flipped to "partner"),
 *     the last state is kept as a GHOST (for handoffWindowSec, or eliminationWindowSec if that is
 *     longer, after that state's time).
 *     When a track becomes reported, it takes over the id of the closest "old" track that passes
 *     the gates below. Old tracks are the ghosts AND the reported tracks that are COASTING right
 *     now (the new track is usually confirmed before the old one has coasted to its timeout).
 *       dt       = t_new - t_old                       in [0, handoffWindowSec]
 *       expected = p_old + v_old * min(dt, handoffLeadSec)
 *       |p_new - expected| <= handoffRadius            (proximity)
 *       |v_new - v_old|    <= handoffSpeedDiff         (velocity)
 *     A coasting track whose id is taken over is deleted: it was a stale copy of the same robot.
 *     So when the guarded robot's track dies and a new one is born in the same place, the id (and
 *     with it the focus lock and the pin timer in RulesGuard) carries over.
 *
 *   By elimination (optional, opponentCount > 0): a robot that was out of view for seconds comes
 *   back far from where it was lost. If a new reportable track got no id by proximity, and
 *     (reported tracks, counting the new one) <= opponentCount
 *     and exactly ONE lost id is younger than eliminationWindowSec,
 *   that lost id must belong to it: there is nobody else it can be.
 * </pre>
 *
 * <p>Zero dependencies and no knowledge of the field or of our own pose, like the rest of
 * phantom.core.tracking.
 */
public final class MultiTargetTracker implements OpponentTracker {
  /**
   * Cost that marks a (track, detection) pair as not allowed. Far above any real cost (those stay
   * within about +-50), so the assignment never prefers a forbidden pair to a real one.
   */
  public static final double FORBIDDEN = 1e9;

  /** Velocity variance of a new track, (2 m/s)^2: the same start as KalmanOpponentTracker. */
  public static final double INITIAL_VELOCITY_VAR = 4.0;

  /** A track is reported while its P(opponent) is above this. */
  public static final double REPORT_PROBABILITY = 0.5;

  /**
   * Smallest allowed measurementStd (1 mm). With R = 0 the innovation variance S could be 0 and
   * the Kalman gain 0/0 = NaN; no camera is this precise anyway.
   */
  public static final double MIN_MEASUREMENT_STD = 1e-3;

  /** Where a track is in its life. */
  public enum Stage {
    /** New, not yet seen often enough to be believed. Never reported. */
    TENTATIVE,
    /** Believed, and seen in the latest frame. */
    CONFIRMED,
    /** Believed, but not seen in the latest frame: its position is an extrapolation. */
    COASTING
  }

  /**
   * Tuning. Use {@link #defaults(double, double)} and the {@code with...} methods.
   *
   * @param measurementStd expected position noise of one detection (m), > 0
   * @param accelStd how hard robots can change velocity, as a std (m/s^2)
   * @param coastTimeoutSec delete a track after this long without a detection (s)
   * @param confirmHits M: detections needed to confirm a new track
   * @param confirmWindow N: frames a new track has to collect them
   * @param colorErrorRate assumed chance that one color label is wrong, 0..0.5
   * @param priorOpponent P(opponent) of a brand-new track before any label, in (0, 1)
   * @param handoffWindowSec how long after its last state a lost id can be taken over by a nearby
   *     track (s); 0 turns proximity re-acquisition off
   * @param handoffRadius proximity gate for re-acquisition (m)
   * @param handoffSpeedDiff velocity gate for re-acquisition (m/s); infinity disables it
   * @param handoffLeadSec extrapolate a lost track's position at most this far ahead (s)
   * @param opponentCount how many opponents are on the field (3 in FRC); 0 turns re-acquisition
   *     by elimination off
   * @param eliminationWindowSec how long a lost id can be taken over by elimination (s)
   * @param confidenceSigma position uncertainty (m, 1 sigma) at which confidence has halved
   * @param firstTrackId id given to the first track; later tracks count up from it
   */
  public record Config(
      double measurementStd,
      double accelStd,
      double coastTimeoutSec,
      int confirmHits,
      int confirmWindow,
      double colorErrorRate,
      double priorOpponent,
      double handoffWindowSec,
      double handoffRadius,
      double handoffSpeedDiff,
      double handoffLeadSec,
      int opponentCount,
      double eliminationWindowSec,
      double confidenceSigma,
      int firstTrackId) {

    public Config {
      if (!(measurementStd >= MIN_MEASUREMENT_STD) || !Double.isFinite(measurementStd)) {
        throw new IllegalArgumentException(
            "measurementStd must be at least " + MIN_MEASUREMENT_STD + " m (it scales the gate)");
      }
      if (!(accelStd >= 0.0) || !Double.isFinite(accelStd)) {
        throw new IllegalArgumentException("accelStd must be >= 0");
      }
      if (!(coastTimeoutSec > 0.0)) {
        throw new IllegalArgumentException("coastTimeoutSec must be > 0");
      }
      if (confirmHits < 1 || confirmWindow < confirmHits) {
        throw new IllegalArgumentException("need 1 <= confirmHits <= confirmWindow");
      }
      if (!(colorErrorRate >= 0.0) || !(colorErrorRate <= 0.5)) {
        throw new IllegalArgumentException("colorErrorRate must be in [0, 0.5]");
      }
      if (!(priorOpponent > 0.0) || !(priorOpponent < 1.0)) {
        throw new IllegalArgumentException("priorOpponent must be strictly between 0 and 1");
      }
      if (!(handoffWindowSec >= 0.0) || !(handoffRadius >= 0.0) || !(handoffSpeedDiff >= 0.0)
          || !(handoffLeadSec >= 0.0) || opponentCount < 0 || !(eliminationWindowSec >= 0.0)) {
        throw new IllegalArgumentException("re-acquisition settings must be >= 0");
      }
      if (!(confidenceSigma > 0.0) || !Double.isFinite(confidenceSigma)) {
        throw new IllegalArgumentException("confidenceSigma must be > 0");
      }
      if (firstTrackId < 0) {
        throw new IllegalArgumentException("track ids must be >= 0 (-1 means 'auto' elsewhere)");
      }
    }

    /**
     * Default tuning. Motion model and timeout match {@link KalmanOpponentTracker#defaults}
     * (accel 4 m/s^2, drop after 1 s). 3-of-5 confirmation. Re-acquisition, chosen with SimMain's
     * {@code --tracker-sweep} (docs/findings/anonymous-tracking.md):
     * <ul>
     *   <li>proximity: within 2 s of the last sighting, inside 2.5 m of where the lost track would
     *       be after at most 0.5 s more of its last velocity, no velocity-difference gate (a 3 m/s
     *       gate rejected good handoffs: a 3-frame-old track's velocity is too noisy);
     *   <li>elimination: 3 opponents (FRC alliances are 3 robots), lost ids kept 4 s (8 s was
     *       worse).
     * </ul>
     */
    public static Config defaults(double measurementStd, double colorErrorRate) {
      return new Config(measurementStd, 4.0, 1.0, 3, 5, colorErrorRate, 0.5,
          2.0, 2.5, Double.POSITIVE_INFINITY, 0.5, 3, 4.0, 0.5, 1);
    }

    public Config withConfirmation(int hits, int window) {
      return new Config(measurementStd, accelStd, coastTimeoutSec, hits, window, colorErrorRate,
          priorOpponent, handoffWindowSec, handoffRadius, handoffSpeedDiff, handoffLeadSec,
          opponentCount, eliminationWindowSec, confidenceSigma, firstTrackId);
    }

    public Config withMotion(double newAccelStd, double newCoastTimeoutSec) {
      return new Config(measurementStd, newAccelStd, newCoastTimeoutSec, confirmHits,
          confirmWindow, colorErrorRate, priorOpponent, handoffWindowSec, handoffRadius,
          handoffSpeedDiff, handoffLeadSec, opponentCount, eliminationWindowSec, confidenceSigma,
          firstTrackId);
    }

    public Config withPriorOpponent(double prior) {
      return new Config(measurementStd, accelStd, coastTimeoutSec, confirmHits, confirmWindow,
          colorErrorRate, prior, handoffWindowSec, handoffRadius, handoffSpeedDiff,
          handoffLeadSec, opponentCount, eliminationWindowSec, confidenceSigma, firstTrackId);
    }

    /** Proximity re-acquisition settings (see the class comment). */
    public Config withHandoff(double windowSec, double radius, double speedDiff, double leadSec) {
      return new Config(measurementStd, accelStd, coastTimeoutSec, confirmHits, confirmWindow,
          colorErrorRate, priorOpponent, windowSec, radius, speedDiff, leadSec, opponentCount,
          eliminationWindowSec, confidenceSigma, firstTrackId);
    }

    /**
     * Re-acquisition by elimination (see the class comment).
     *
     * @param opponents how many opponents are on the field; 0 = off
     * @param windowSec how long a lost id stays available to it (s)
     */
    public Config withElimination(int opponents, double windowSec) {
      return new Config(measurementStd, accelStd, coastTimeoutSec, confirmHits, confirmWindow,
          colorErrorRate, priorOpponent, handoffWindowSec, handoffRadius, handoffSpeedDiff,
          handoffLeadSec, opponents, windowSec, confidenceSigma, firstTrackId);
    }

    /** Every re-born track gets a brand-new id (for measuring what re-acquisition buys). */
    public Config withoutHandoff() {
      return withHandoff(0.0, handoffRadius, handoffSpeedDiff, handoffLeadSec)
          .withElimination(0, 0.0);
    }

    /** Same settings, track ids counting up from {@code id}. */
    public Config withFirstTrackId(int id) {
      return new Config(measurementStd, accelStd, coastTimeoutSec, confirmHits, confirmWindow,
          colorErrorRate, priorOpponent, handoffWindowSec, handoffRadius, handoffSpeedDiff,
          handoffLeadSec, opponentCount, eliminationWindowSec, confidenceSigma, id);
    }

    /** How long a lost id is remembered at all (s). */
    double ghostLifetimeSec() {
      return Math.max(handoffWindowSec, opponentCount > 0 ? eliminationWindowSec : 0.0);
    }
  }

  /**
   * Read-only look at one track, including the ones that are not reported (tentative tracks and
   * robots believed to be partners). For the sim display, metrics and tests.
   *
   * @param id track id (only meaningful to the defense code while {@code reported})
   * @param position filtered position at {@code timestampSec} (m)
   * @param velocity filtered velocity (m/s)
   * @param timestampSec capture time of the last detection used (s)
   * @param probabilityOpponent color belief, 0..1
   * @param reported true if getOpponents currently returns this track
   */
  public record TrackView(
      int id,
      Stage stage,
      Vec2 position,
      Vec2 velocity,
      double timestampSec,
      double probabilityOpponent,
      boolean reported) {}

  private static final class Track {
    int id;
    final AxisKalman x;
    final AxisKalman y;
    double time;
    Stage stage = Stage.TENTATIVE;
    final MOfN confirmation;
    final ColorBelief color;
    boolean reported = false;

    Track(int id, Vec2 z, double time, double r, Config c) {
      this.id = id;
      this.x = new AxisKalman(z.x(), 0.0, r, INITIAL_VELOCITY_VAR);
      this.y = new AxisKalman(z.y(), 0.0, r, INITIAL_VELOCITY_VAR);
      this.time = time;
      this.confirmation = new MOfN(c.confirmHits(), c.confirmWindow());
      this.color = new ColorBelief(c.priorOpponent());
    }

    Vec2 position() {
      return new Vec2(x.position(), y.position());
    }

    Vec2 velocity() {
      return new Vec2(x.velocity(), y.velocity());
    }
  }

  /** The last state of a reported id that lost its track. */
  private record Ghost(int id, Vec2 position, Vec2 velocity, double time) {}

  private final DetectionSource source;
  private final Config config;
  private final double r;
  private final List<Track> tracks = new ArrayList<>();
  private final List<Ghost> ghosts = new ArrayList<>();
  private int nextId;
  private double lastFrameTime = Double.NEGATIVE_INFINITY;
  private int framesProcessed = 0;
  private int framesDropped = 0;
  private int detectionsIgnored = 0;
  private int handoffs = 0;
  private int eliminations = 0;

  public MultiTargetTracker(DetectionSource source, Config config) {
    this.source = source;
    this.config = config;
    this.r = config.measurementStd() * config.measurementStd();
    this.nextId = config.firstTrackId();
  }

  /**
   * @param measurementStd expected position noise of one detection (m)
   * @param colorErrorRate assumed chance that one color label is wrong (0..0.5)
   */
  public static MultiTargetTracker defaults(
      DetectionSource source, double measurementStd, double colorErrorRate) {
    return new MultiTargetTracker(source, Config.defaults(measurementStd, colorErrorRate));
  }

  @Override
  public List<TrackedRobot> getOpponents(double nowSec) {
    List<DetectionFrame> polled = source.poll(nowSec);
    if (polled != null && !polled.isEmpty()) {
      List<DetectionFrame> frames = new ArrayList<>(polled.size());
      for (DetectionFrame f : polled) {
        if (f != null) {
          frames.add(f);
        }
      }
      frames.sort(Comparator.comparingDouble(DetectionFrame::timestampSec));
      for (DetectionFrame f : frames) {
        process(f);
      }
    }
    expire(nowSec);

    List<TrackedRobot> out = new ArrayList<>();
    for (Track t : tracks) {
      if (t.reported) {
        out.add(new TrackedRobot(t.id, t.position(), t.velocity(), t.time, confidence(t, nowSec)));
      }
    }
    out.sort(Comparator.comparingInt(TrackedRobot::id));
    return out;
  }

  /** One camera frame: associate, update, start new tracks, update the life cycles. */
  private void process(DetectionFrame frame) {
    double t = frame.timestampSec();
    if (!Double.isFinite(t) || t <= lastFrameTime) {
      framesDropped++; // out of order or duplicate
      return;
    }
    expire(t); // a track that timed out before this frame was captured cannot claim from it

    List<Detection> dets = new ArrayList<>();
    for (Detection d : frame.detections()) {
      if (d != null && d.position() != null && d.position().isFinite() && d.label() != null) {
        dets.add(d);
      } else {
        detectionsIgnored++; // never let a NaN position into a filter
      }
    }

    int nTracks = tracks.size();
    int nDets = dets.size();
    int[] detOfTrack = new int[nTracks];
    Arrays.fill(detOfTrack, -1);
    if (nTracks > 0 && nDets > 0) {
      double[][] cost = new double[nTracks][nDets];
      for (int i = 0; i < nTracks; i++) {
        Track tr = tracks.get(i);
        double dt = t - tr.time;
        Vec2 predicted = new Vec2(tr.x.predictedPosition(dt), tr.y.predictedPosition(dt));
        double sx = tr.x.predictedPositionVar(dt, config.accelStd()) + r;
        double sy = tr.y.predictedPositionVar(dt, config.accelStd()) + r;
        for (int j = 0; j < nDets; j++) {
          Detection d = dets.get(j);
          double d2 = Gating.mahalanobisSquared(d.position().minus(predicted), sx, sy);
          double c =
              Gating.accepts(d2)
                  ? associationCost(
                      d2, sx, sy, tr.color.likelihood(d.label(), config.colorErrorRate()))
                  : FORBIDDEN;
          cost[i][j] = Double.isFinite(c) ? c : FORBIDDEN;
        }
      }
      int[] assigned = Hungarian.solve(cost);
      for (int i = 0; i < nTracks; i++) {
        if (assigned[i] >= 0 && cost[i][assigned[i]] < FORBIDDEN / 2.0) {
          detOfTrack[i] = assigned[i];
        }
      }
    }

    boolean[] detUsed = new boolean[nDets];
    List<Track> rejected = new ArrayList<>();
    for (int i = 0; i < nTracks; i++) {
      Track tr = tracks.get(i);
      int j = detOfTrack[i];
      if (j >= 0) {
        detUsed[j] = true;
        Detection d = dets.get(j);
        double dt = t - tr.time;
        tr.x.predict(dt, config.accelStd());
        tr.y.predict(dt, config.accelStd());
        tr.x.update(d.position().x(), r);
        tr.y.update(d.position().y(), r);
        tr.time = t;
        tr.color.update(d.label(), config.colorErrorRate());
        if (tr.stage == Stage.TENTATIVE) {
          if (tr.confirmation.add(true) == MOfN.Decision.CONFIRMED) {
            tr.stage = Stage.CONFIRMED;
          }
        } else {
          tr.stage = Stage.CONFIRMED;
        }
      } else if (tr.stage == Stage.TENTATIVE) {
        if (tr.confirmation.add(false) == MOfN.Decision.REJECTED) {
          rejected.add(tr);
        }
      } else {
        tr.stage = Stage.COASTING;
      }
    }
    tracks.removeAll(rejected);

    for (int j = 0; j < nDets; j++) {
      if (!detUsed[j]) {
        Detection d = dets.get(j);
        Track born = new Track(nextId++, d.position(), t, r, config);
        born.color.update(d.label(), config.colorErrorRate());
        if (born.confirmation.add(true) == MOfN.Decision.CONFIRMED) {
          born.stage = Stage.CONFIRMED; // only when confirmHits = 1
        }
        tracks.add(born);
      }
    }

    lastFrameTime = t;
    framesProcessed++;
    updateReporting();
  }

  /**
   * Association cost = -2 ln(likelihood) without its constants (see class comment):
   *
   * <pre>
   *   c = d2 + ln(Sx * Sy) - 2 ln(labelLikelihood)
   * </pre>
   */
  public static double associationCost(double d2, double sx, double sy, double labelLikelihood) {
    return d2 + Math.log(sx * sy) - 2.0 * Math.log(labelLikelihood);
  }

  /** Start or stop reporting tracks whose status changed, handing ids over where possible. */
  private void updateReporting() {
    List<Track> newlyReportable = new ArrayList<>();
    for (Track tr : tracks) {
      boolean should =
          tr.stage != Stage.TENTATIVE && tr.color.probabilityOpponent() > REPORT_PROBABILITY;
      if (should && !tr.reported) {
        newlyReportable.add(tr);
      } else if (!should && tr.reported) {
        // Believed to be a partner now: the id is released (someone else may be the real one).
        remember(tr);
        tr.id = nextId++;
        tr.reported = false;
      }
    }
    handOff(newlyReportable);
    for (Track tr : newlyReportable) {
      tr.reported = true;
    }
  }

  private void remember(Track tr) {
    if (config.ghostLifetimeSec() > 0.0) {
      ghosts.add(new Ghost(tr.id, tr.position(), tr.velocity(), tr.time));
    }
  }

  /**
   * Give newly reportable tracks the ids of nearby lost (ghost) or coasting reported tracks,
   * closest pairs first. A coasting track whose id is taken over is deleted.
   */
  private void handOff(List<Track> newlyReportable) {
    if (config.ghostLifetimeSec() <= 0.0 || newlyReportable.isEmpty()) {
      return;
    }
    record Candidate(Track track, Ghost ghost, Track coasting, double error) {}
    List<Candidate> candidates = new ArrayList<>();
    for (Track tr : newlyReportable) {
      for (Ghost g : ghosts) {
        double e = handoffError(tr.position(), tr.velocity(), tr.time, g.position(), g.velocity(),
            g.time(), config);
        if (Double.isFinite(e)) {
          candidates.add(new Candidate(tr, g, null, e));
        }
      }
      for (Track old : tracks) {
        if (old.reported && old.stage == Stage.COASTING) {
          double e = handoffError(tr.position(), tr.velocity(), tr.time, old.position(),
              old.velocity(), old.time, config);
          if (Double.isFinite(e)) {
            candidates.add(new Candidate(tr, null, old, e));
          }
        }
      }
    }
    candidates.sort(Comparator.comparingDouble(Candidate::error));
    List<Track> done = new ArrayList<>();
    List<Track> merged = new ArrayList<>();
    for (Candidate c : candidates) {
      if (done.contains(c.track())
          || (c.ghost() != null && !ghosts.contains(c.ghost()))
          || (c.coasting() != null && merged.contains(c.coasting()))) {
        continue;
      }
      if (c.ghost() != null) {
        c.track().id = c.ghost().id();
        ghosts.remove(c.ghost());
      } else {
        c.track().id = c.coasting().id;
        merged.add(c.coasting());
      }
      done.add(c.track());
      handoffs++;
    }
    tracks.removeAll(merged);
    eliminate(newlyReportable, done);
  }

  /** Re-acquisition by elimination (see the class comment) for tracks that got no id yet. */
  private void eliminate(List<Track> newlyReportable, List<Track> done) {
    if (config.opponentCount() <= 0) {
      return;
    }
    int reportedAfter = newlyReportable.size();
    for (Track tr : tracks) {
      if (tr.reported) {
        reportedAfter++;
      }
    }
    if (reportedAfter > config.opponentCount()) {
      return; // more opponent tracks than opponents: a duplicate or a phantom is among them
    }
    for (Track tr : newlyReportable) {
      if (done.contains(tr)) {
        continue;
      }
      Ghost only = null;
      int candidates = 0;
      for (Ghost g : ghosts) {
        double dt = tr.time - g.time();
        if (dt >= 0.0 && dt <= config.eliminationWindowSec()) {
          candidates++;
          only = g;
        }
      }
      if (candidates == 1) {
        tr.id = only.id();
        ghosts.remove(only);
        done.add(tr);
        eliminations++;
      }
    }
  }

  /**
   * Re-acquisition test for one (new track, old track) pair. Returns the position error if the
   * pair passes every gate, otherwise +infinity:
   *
   * <pre>
   *   dt       = tNew - tOld                          must be in [0, handoffWindowSec]
   *   expected = pOld + vOld * min(dt, handoffLeadSec)
   *   error    = |pNew - expected|                    must be <= handoffRadius     (proximity)
   *   |vNew - vOld|                                   must be <= handoffSpeedDiff  (velocity)
   * </pre>
   */
  public static double handoffError(
      Vec2 pNew, Vec2 vNew, double tNew, Vec2 pOld, Vec2 vOld, double tOld, Config c) {
    double dt = tNew - tOld;
    if (!(dt >= 0.0) || dt > c.handoffWindowSec()) {
      return Double.POSITIVE_INFINITY;
    }
    double error = handoffPositionError(pNew, pOld, vOld, dt, c.handoffLeadSec());
    double speedDiff = vNew.minus(vOld).norm();
    if (!(error <= c.handoffRadius()) || !(speedDiff <= c.handoffSpeedDiff())) {
      return Double.POSITIVE_INFINITY;
    }
    return error;
  }

  /**
   * Distance between a new track and where a lost one should be by now:
   *
   * <pre>
   *   expected = p_ghost + v_ghost * min(dt, leadSec)
   *   error    = |p_track - expected|
   * </pre>
   */
  public static double handoffPositionError(
      Vec2 trackPosition, Vec2 ghostPosition, Vec2 ghostVelocity, double dt, double leadSec) {
    Vec2 expected = ghostPosition.plus(ghostVelocity.times(Math.min(dt, leadSec)));
    return trackPosition.distanceTo(expected);
  }

  /** Drop tracks not seen for coastTimeoutSec and ghosts older than the handoff window. */
  private void expire(double nowSec) {
    for (int i = tracks.size() - 1; i >= 0; i--) {
      Track tr = tracks.get(i);
      if (nowSec - tr.time > config.coastTimeoutSec()) {
        if (tr.reported) {
          remember(tr);
        }
        tracks.remove(i);
      }
    }
    ghosts.removeIf(g -> nowSec - g.time() > config.ghostLifetimeSec());
  }

  private double confidence(Track tr, double nowSec) {
    double dt = Math.max(0.0, nowSec - tr.time);
    double var =
        (tr.x.predictedPositionVar(dt, config.accelStd())
                + tr.y.predictedPositionVar(dt, config.accelStd()))
            / 2.0;
    return confidence(tr.color.probabilityOpponent(), var, config.confidenceSigma());
  }

  /**
   * Confidence of a reported track (see class comment):
   *
   * <pre>
   *   confidence = P(opponent) * sigma0^2 / (sigma0^2 + positionVar)
   * </pre>
   *
   * @return a value in [0, 1]; 0 if the inputs are not usable numbers
   */
  public static double confidence(double probabilityOpponent, double positionVar, double sigma0) {
    double s2 = sigma0 * sigma0;
    double c = probabilityOpponent * s2 / (s2 + Math.max(0.0, positionVar));
    return Double.isNaN(c) ? 0.0 : Math.max(0.0, Math.min(1.0, c));
  }

  /** Every live track, reported or not, ordered by id. */
  public List<TrackView> tracks() {
    List<TrackView> out = new ArrayList<>();
    for (Track tr : tracks) {
      out.add(new TrackView(tr.id, tr.stage, tr.position(), tr.velocity(), tr.time,
          tr.color.probabilityOpponent(), tr.reported));
    }
    out.sort(Comparator.comparingInt(TrackView::id));
    return out;
  }

  public Config config() {
    return config;
  }

  /** Camera frames used so far. */
  public int framesProcessed() {
    return framesProcessed;
  }

  /** Frames thrown away because they were not newer than the last processed frame. */
  public int framesDropped() {
    return framesDropped;
  }

  /** Detections thrown away because their position was not a finite number. */
  public int detectionsIgnored() {
    return detectionsIgnored;
  }

  /** Times a new track took over the id of a lost or coasting one by proximity. */
  public int handoffs() {
    return handoffs;
  }

  /** Times a new track took over a lost id by elimination. */
  public int eliminations() {
    return eliminations;
  }
}
