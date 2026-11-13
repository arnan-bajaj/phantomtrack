package phantom.core.modes;

import phantom.core.geometry.Vec2;

/**
 * What a defense mode wants the robot to do right now.
 *
 * @param position where to be (m, field coordinates)
 * @param heading which way to face (rad)
 * @param velocityFeedforward how fast the target point itself is moving (m/s); lets the controller
 *     keep up with a moving opponent instead of always lagging behind
 * @param opponentId the opponent this target is reacting to, or -1 for none
 */
public record DefenseTarget(Vec2 position, double heading, Vec2 velocityFeedforward, int opponentId) {}
