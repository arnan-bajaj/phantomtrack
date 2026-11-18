package phantom.core.tracking;

/**
 * Which alliance a camera pipeline thinks a detected robot is on, judged from its bumper color.
 * The pipeline knows our own alliance color, so it reports "opponent" or "partner" instead of
 * red / blue. The label is sometimes wrong; {@link ColorBelief} deals with that.
 */
public enum AllianceLabel {
  /** Bumpers look like the other alliance's color. */
  OPPONENT,
  /** Bumpers look like our own alliance's color. */
  PARTNER,
  /** The pipeline could not tell (bumpers hidden, bad light). Carries no information. */
  UNKNOWN
}
