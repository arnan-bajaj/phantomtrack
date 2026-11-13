package phantom.core.tracking;

import java.util.List;

/**
 * The single boundary between "where opponent data comes from" and the defense code.
 *
 * <p>Implementations: perfect sim truth (now), noisy sim (Phase 2), NetworkTables from a second
 * robot (Phase 7), vision (Phase 8). Nothing in phantom.core.modes or phantom.core.control may
 * know which one is in use.
 */
public interface OpponentTracker {
  /** Current opponent tracks. Never null; may be empty. */
  List<TrackedRobot> getOpponents(double nowSec);
}
