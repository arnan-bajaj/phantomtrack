package phantom.core.rules;

/**
 * Game-manual pinning limits. Kept as data so the 2027 numbers can be dropped in at kickoff.
 *
 * @param limitSeconds maximum allowed contact time in one pin (s)
 * @param resetSeparation bumper-to-bumper gap that ends a pin (m)
 */
public record PinRules(double limitSeconds, double resetSeparation) {

  /**
   * PLACEHOLDER based on recent FRC games (3 s, about 6 ft). Confirm both numbers in the 2027 game
   * manual after kickoff.
   */
  public static PinRules placeholder() {
    return new PinRules(3.0, 1.83);
  }
}
