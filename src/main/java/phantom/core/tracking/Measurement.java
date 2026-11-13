package phantom.core.tracking;

import phantom.core.geometry.Vec2;

/**
 * One raw sighting of an opponent, as a camera pipeline would report it.
 *
 * @param id which opponent (assumes identity is known, e.g. from bumper numbers)
 * @param position measured field position (m), noisy
 * @param timestampSec when the image was captured (s), which is earlier than when it arrives
 */
public record Measurement(int id, Vec2 position, double timestampSec) {}
