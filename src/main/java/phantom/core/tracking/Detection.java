package phantom.core.tracking;

import phantom.core.geometry.Vec2;

/**
 * One anonymous robot sighting from a camera frame: a position and a bumper-color guess, and
 * nothing else. Unlike {@link Measurement} it does NOT say which robot it is. It may be an
 * opponent, an alliance partner, or not a robot at all (a false positive).
 *
 * @param position measured field position (m), noisy
 * @param label the pipeline's alliance guess from bumper color (may be wrong)
 */
public record Detection(Vec2 position, AllianceLabel label) {}
