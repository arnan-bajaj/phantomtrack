package frc.robot.defense;

import java.util.List;
import phantom.core.geometry.Vec2;
import phantom.core.rules.PinTimer;
import phantom.sim.Contact;

/**
 * Lets a ghost "push back". A real robot cannot feel a ghost, so without this the assist would
 * drive straight through one. Here the simulator's pushing-match model ({@link Contact}) decides
 * how fast the pair would move, and the assist command is limited to that.
 *
 * <pre>
 *   for each ghost i with  |p_i - p_self| - 2 * robotHalfSize <= PinTimer.CONTACT_TOLERANCE:
 *     n      = unit(p_i - p_self)                  ((1, 0) if the centers coincide, as in the sim)
 *     e_self = Contact.effort(v_cmd, n, speedCap)  (+1 = we drive into it at the speed cap)
 *     e_i    = Contact.effort(vDesired_i, -n, vMax_i)   (+1 = it drives into us flat out)
 *     v_pair = Contact.contactSpeed(e_self, e_i) * speedCap / SIM_TOP_SPEED
 *     if v . n > v_pair:   v = v - (v . n - v_pair) * n
 * </pre>
 *
 * Contact.PUSH_SPEED (2.0 m/s) was tuned for 4.5 m/s robots, so it is scaled to our speed cap:
 * pushing a parked ghost flat out moves the pair at 2.0 / 2 / 4.5 = 22% of top speed, in the sim
 * and here. Both pushing flat out gives v_pair = 0: a stall, like a real pushing match.
 *
 * <p>Two-sided, like the sim: when the ghost pushes harder, v_pair is negative and the assist
 * gives way, backing off along n at that speed. This only ever changes the ASSIST's command, which
 * still goes through the speed cap, the acceleration limit and the geofence afterwards. While the
 * driver drives, ghosts cannot move the robot at all (SimWorld.stepWithRealSelf just blocks
 * them). Motion away from or along the ghost faster than v_pair is untouched.
 */
public final class VirtualContact {
  private VirtualContact() {}

  /** Top speed of the simulator's robots (RobotBody.typical), which Contact was tuned against. */
  public static final double SIM_TOP_SPEED = 4.5;

  /**
   * One ghost as the pushing model needs it.
   *
   * @param position center (m)
   * @param desiredVelocity the velocity it is trying to drive at (m/s)
   * @param maxSpeed its top speed (m/s)
   */
  public record Ghost(Vec2 position, Vec2 desiredVelocity, double maxSpeed) {}

  /**
   * @param command the assist's velocity command (m/s, blue frame)
   * @param self our robot's center (m)
   * @param speedCap the assist's speed cap (m/s); driving at the cap counts as full effort
   * @return the command with motion into touching ghosts limited; never NaN
   */
  public static Vec2 limit(
      Vec2 command, Vec2 self, List<Ghost> ghosts, double robotHalfSize, double speedCap) {
    if (!command.isFinite() || !self.isFinite() || !(speedCap > 0.0)) {
      return Vec2.ZERO;
    }
    Vec2 v = command;
    for (Ghost g : ghosts) {
      Vec2 delta = g.position().minus(self);
      double dist = delta.norm();
      if (dist - 2.0 * robotHalfSize > PinTimer.CONTACT_TOLERANCE) {
        continue;
      }
      Vec2 n = dist < Vec2.EPS ? new Vec2(1.0, 0.0) : delta.times(1.0 / dist);
      double effortSelf = Contact.effort(command, n, speedCap);
      double effortGhost = Contact.effort(g.desiredVelocity(), n.times(-1.0), g.maxSpeed());
      double pairSpeed = Contact.contactSpeed(effortSelf, effortGhost) * speedCap / SIM_TOP_SPEED;
      double into = v.dot(n);
      if (into > pairSpeed) {
        v = v.minus(n.times(into - pairSpeed));
      }
    }
    return v;
  }
}
