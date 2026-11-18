package phantom.sim;

import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/**
 * Simple kinematic swerve robot: follows the commanded velocity, limited by max speed and max
 * acceleration. Good enough to develop defense algorithms; not a physics engine.
 */
public final class RobotBody {
  private final int id;
  private final double maxSpeed;
  private final double maxAccel;
  private final double maxOmega;
  private final double maxAlpha;

  private Vec2 position;
  private Vec2 velocity = Vec2.ZERO;
  private Vec2 desiredVelocity = Vec2.ZERO; // last command, used by the pushing model
  private Vec2 velocityBeforeStep = Vec2.ZERO; // for the acceleration over the last step
  private Vec2 acceleration = Vec2.ZERO;
  private double heading;
  private double omega = 0.0;
  private double desiredOmega = 0.0;

  /**
   * @param maxSpeed m/s
   * @param maxAccel m/s^2
   * @param maxOmega rad/s
   * @param maxAlpha rad/s^2
   */
  public RobotBody(
      int id, Pose start, double maxSpeed, double maxAccel, double maxOmega, double maxAlpha) {
    this.id = id;
    this.position = start.position();
    this.heading = start.heading();
    this.maxSpeed = maxSpeed;
    this.maxAccel = maxAccel;
    this.maxOmega = maxOmega;
    this.maxAlpha = maxAlpha;
  }

  /** Typical competitive swerve: 4.5 m/s, 6 m/s^2. */
  public static RobotBody typical(int id, Pose start) {
    return new RobotBody(id, start, 4.5, 6.0, 2.0 * Math.PI, 4.0 * Math.PI);
  }

  /** Advance one timestep toward the commanded (field-relative) velocity. */
  public void step(ChassisCommand cmd, double dt, FieldConfig field) {
    // Translation: move velocity toward the command, at most maxAccel * dt per step.
    Vec2 desired = cmd.velocity().clampNorm(maxSpeed);
    desiredVelocity = desired;
    velocityBeforeStep = velocity;
    Vec2 dv = desired.minus(velocity).clampNorm(maxAccel * dt);
    velocity = velocity.plus(dv).clampNorm(maxSpeed);

    // Rotation: same idea.
    desiredOmega = Math.max(-maxOmega, Math.min(maxOmega, cmd.omega()));
    double dOmega = Math.max(-maxAlpha * dt, Math.min(maxAlpha * dt, desiredOmega - omega));
    omega += dOmega;

    position = position.plus(velocity.times(dt));
    heading = Angles.wrap(heading + omega * dt);
    keepInside(field);
    finishStep(dt);
  }

  /**
   * Records the acceleration over the step that just ended, like an accelerometer would feel it:
   *
   * <pre>
   *   a = (v_end - v_start) / dt
   * </pre>
   *
   * {@link #step} calls this itself; SimWorld calls it again after collisions have changed the
   * velocity, so hitting a wall or a robot shows up as a large deceleration.
   */
  public void finishStep(double dt) {
    acceleration = dt > 0.0 ? velocity.minus(velocityBeforeStep).times(1.0 / dt) : Vec2.ZERO;
  }

  /** Stop at the field walls: clamp position and kill velocity into the wall. */
  public void keepInside(FieldConfig field) {
    Vec2 clamped = field.clampInside(position);
    double vx = velocity.x();
    double vy = velocity.y();
    if (clamped.x() != position.x()) {
      vx = 0.0;
    }
    if (clamped.y() != position.y()) {
      vy = 0.0;
    }
    position = clamped;
    velocity = new Vec2(vx, vy);
  }

  public int id() {
    return id;
  }

  public Pose pose() {
    return new Pose(position, heading);
  }

  public Vec2 position() {
    return position;
  }

  public Vec2 velocity() {
    return velocity;
  }

  /** The velocity this robot is trying to drive at (its last command, speed-capped). */
  public Vec2 desiredVelocity() {
    return desiredVelocity;
  }

  /** The turn rate this robot is trying to reach (its last command, rate-capped; rad/s). */
  public double desiredOmega() {
    return desiredOmega;
  }

  /** Turn rate (rad/s, CCW positive). */
  public double omega() {
    return omega;
  }

  /** Acceleration over the last step, including the effect of collisions (m/s^2). */
  public Vec2 acceleration() {
    return acceleration;
  }

  public double heading() {
    return heading;
  }

  public double maxSpeed() {
    return maxSpeed;
  }

  public double maxAccel() {
    return maxAccel;
  }

  /**
   * Phase 7 (ghost opponents on a real robot): make this body mirror a REAL robot. Replaces the
   * whole state with what the drivetrain measured, instead of integrating a command. Used by
   * {@link SimWorld#stepWithRealSelf}.
   *
   * @param measuredVelocity the real robot's field velocity (m/s)
   * @param desired the velocity the real robot is being asked to drive at (m/s); the pushing model
   *     and the pin referee read it as "how hard are we driving into them"
   */
  void setMeasured(Pose pose, Vec2 measuredVelocity, Vec2 desired) {
    position = pose.position();
    heading = Angles.wrap(pose.heading());
    velocityBeforeStep = velocity; // so finishStep gives (measured now - measured last) / dt
    velocity = measuredVelocity;
    desiredVelocity = desired.clampNorm(maxSpeed);
  }

  // Used by collision resolution only.
  void setPosition(Vec2 p) {
    position = p;
  }

  void setVelocity(Vec2 v) {
    velocity = v;
  }
}
