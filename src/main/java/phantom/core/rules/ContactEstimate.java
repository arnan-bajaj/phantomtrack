package phantom.core.rules;

import java.util.Map;
import java.util.Set;
import phantom.core.geometry.Vec2;

/**
 * What {@link ContactDetector} believes right now.
 *
 * @param inContact true if we are touching something (after enter/exit hysteresis)
 * @param probability in-contact probability, 0..1 (the highest of the per-opponent values)
 * @param direction unit vector from us toward what we touch; ZERO if not in contact
 * @param opponentId opponent most likely in contact; {@link ContactDetector#UNKNOWN} if we are
 *     touching something we cannot identify, or if we are not in contact at all
 * @param blocked true if the drivetrain alone says we are pushing on something (any robot or a
 *     wall), no camera needed
 * @param pushDirection unit direction we are commanding; ZERO if we are not pushing
 * @param contacts everything we are in contact with (opponent ids, and UNKNOWN for something
 *     unidentified); empty if not in contact
 * @param probabilities in-contact probability per opponent id (and UNKNOWN)
 * @param stallCue 0..1, pushing but not moving
 * @param currentCue 0..1, drive current above threshold
 * @param decelCue 0..1, sudden deceleration (held for a short time after the hit)
 */
public record ContactEstimate(
    boolean inContact,
    double probability,
    Vec2 direction,
    int opponentId,
    boolean blocked,
    Vec2 pushDirection,
    Set<Integer> contacts,
    Map<Integer, Double> probabilities,
    double stallCue,
    double currentCue,
    double decelCue) {

  public ContactEstimate {
    contacts = Set.copyOf(contacts);
    probabilities = Map.copyOf(probabilities);
  }

  /** Nothing touching, nothing known. */
  public static ContactEstimate none() {
    return new ContactEstimate(
        false, 0.0, Vec2.ZERO, ContactDetector.UNKNOWN, false, Vec2.ZERO, Set.of(), Map.of(), 0, 0, 0);
  }

  /** True if we are in contact with this opponent (or with something unidentified, for UNKNOWN). */
  public boolean inContactWith(int id) {
    return contacts.contains(id);
  }
}
