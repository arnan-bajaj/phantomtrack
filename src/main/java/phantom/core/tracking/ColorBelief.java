package phantom.core.tracking;

/**
 * How sure we are that a tracked robot is an opponent, built up from bumper-color labels that are
 * sometimes wrong. Kept as log-odds, so every new label is just an addition.
 *
 * <pre>
 *   L = ln( P(opponent) / P(partner) )                 L = 0 means 50/50
 *   e = chance that one label is wrong (the detector's color error rate)
 *
 *   label OPPONENT:   L = L + ln((1 - e) / e)
 *   label PARTNER:    L = L - ln((1 - e) / e)
 *   label UNKNOWN:    L unchanged
 *   then clamp L to [-MAX_LOG_ODDS, +MAX_LOG_ODDS]
 *
 *   P(opponent) = 1 / (1 + exp(-L))
 *
 *   How likely each label is for this track (used when matching detections to tracks):
 *     P(label = OPPONENT) = P(opp) (1 - e) + (1 - P(opp)) e
 *     P(label = PARTNER)  = P(opp) e       + (1 - P(opp)) (1 - e)
 *     P(label = UNKNOWN)  = 1                                  (says nothing either way)
 * </pre>
 *
 * <p>Why clamp: without it a robot seen 1000 times could never be re-classified. With the clamp
 * at 10 (99.995%) it takes 4 wrong labels in a row at e = 5% (10 / ln 19 = 3.4) or 6 in a row at
 * e = 15% (10 / ln 5.67 = 5.8) to flip a settled belief, which makes random flips rare
 * (0.05^4 = 6e-6 and 0.15^6 = 1e-5 per frame) while a real change of identity (the track slid
 * onto another robot) is still corrected within a few frames.
 *
 * <p>This assumes label errors are independent from frame to frame. A real detector that is
 * consistently wrong about one robot (odd bumper color, bad lighting in one corner) breaks that
 * assumption and will fool this filter.
 */
public final class ColorBelief {
  /** Largest |log-odds| kept. exp(10) = 22026, so the belief saturates at 99.995%. */
  public static final double MAX_LOG_ODDS = 10.0;

  /** No detector label is trusted beyond 99.9%, even if a caller claims a 0% error rate. */
  public static final double MIN_ERROR_RATE = 1e-3;

  private double logOdds;

  /**
   * @param priorOpponentProbability belief before any label, in (0, 1); 0.5 = no idea
   */
  public ColorBelief(double priorOpponentProbability) {
    if (!(priorOpponentProbability > 0.0) || !(priorOpponentProbability < 1.0)) {
      throw new IllegalArgumentException("prior must be strictly between 0 and 1");
    }
    this.logOdds = clamp(Math.log(priorOpponentProbability / (1.0 - priorOpponentProbability)));
  }

  /** The error rate actually used: forced into [MIN_ERROR_RATE, 0.5]. */
  public static double effectiveErrorRate(double errorRate) {
    if (Double.isNaN(errorRate)) {
      return 0.5; // unknown reliability: treat labels as coin flips (no information)
    }
    return Math.max(MIN_ERROR_RATE, Math.min(0.5, errorRate));
  }

  /** Log-odds added by one label: ln((1 - e) / e). Zero when e = 0.5 (labels are coin flips). */
  public static double step(double errorRate) {
    double e = effectiveErrorRate(errorRate);
    return Math.log((1.0 - e) / e);
  }

  /** Fold one color label into the belief. */
  public void update(AllianceLabel label, double errorRate) {
    if (label == AllianceLabel.OPPONENT) {
      logOdds = clamp(logOdds + step(errorRate));
    } else if (label == AllianceLabel.PARTNER) {
      logOdds = clamp(logOdds - step(errorRate));
    }
  }

  public double logOdds() {
    return logOdds;
  }

  /** P(opponent) = 1 / (1 + exp(-L)), in (0, 1). */
  public double probabilityOpponent() {
    return 1.0 / (1.0 + Math.exp(-logOdds));
  }

  /** P(the detector prints this label | what we believe about this track). In (0, 1]. */
  public double likelihood(AllianceLabel label, double errorRate) {
    double e = effectiveErrorRate(errorRate);
    double p = probabilityOpponent();
    if (label == AllianceLabel.OPPONENT) {
      return p * (1.0 - e) + (1.0 - p) * e;
    } else if (label == AllianceLabel.PARTNER) {
      return p * e + (1.0 - p) * (1.0 - e);
    }
    return 1.0;
  }

  private static double clamp(double l) {
    return Math.max(-MAX_LOG_ODDS, Math.min(MAX_LOG_ODDS, l));
  }
}
