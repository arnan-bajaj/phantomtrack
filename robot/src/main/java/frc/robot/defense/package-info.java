/**
 * PhantomTrack's robot-side logic: safety rules, ghost opponents and the per-loop pipeline.
 *
 * <p><b>Nothing in this package may import WPILib</b> (enforced by a test). It uses only {@code
 * phantom.core} types, so it can be unit-tested without a robot or HAL and it survives the WPILib
 * 2027 / SystemCore move untouched. Everything that talks to WPILib lives one package up, in
 * {@code frc.robot}, and is kept as thin as possible.
 *
 * <p>All positions, velocities and commands in here are in the <b>blue frame</b> (WPILib field
 * coordinates as seen when we are the blue alliance). {@code frc.robot.AllianceFrame} converts at
 * the boundary when we are red.
 */
package frc.robot.defense;
