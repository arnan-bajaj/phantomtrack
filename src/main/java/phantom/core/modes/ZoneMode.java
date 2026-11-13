package phantom.core.modes;

import java.util.Optional;
import java.util.OptionalDouble;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Intercept;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Zone defense: guard a circle of radius R around the goal. Wait at an anchor point; when an
 * opponent enters the zone (or is about to), go meet it goal-side, then return to the anchor.
 *
 * <p>Ignores the driver's focus choice: the zone decides who the threat is.
 *
 * <pre>
 *   threat = opponent with |p_opp - goal| <= R + warningMargin, nearest the goal
 *   t*     = Intercept.time(p_opp - p_self, v_opp, ourSpeed)   (smallest positive root)
 *   q      = p_opp + v_opp * t*            (or p_opp if we can never catch it)
 *   p*     = q + min(standoff, |goal - q|) * unit(goal - q),  then kept inside the circle
 *   anchor = goal + anchorDistance * unit(fieldCenter - goal)
 * </pre>
 */
public final class ZoneMode implements DefenseMode {
  private final double radius;
  private final double warningMargin;
  private final double anchorDistance;
  private final double standoff;
  private final double ourSpeed;

  /**
   * @param radius zone radius around the goal (m)
   * @param warningMargin start reacting when an opponent is this far outside the zone (m)
   * @param anchorDistance resting point distance from the goal, toward field center (m)
   * @param standoff how far goal-side of the intercept point to stand (m)
   * @param ourSpeed our top speed, for intercept timing (m/s)
   */
  public ZoneMode(
      double radius, double warningMargin, double anchorDistance, double standoff, double ourSpeed) {
    if (radius <= 0 || warningMargin < 0 || anchorDistance < 0 || standoff < 0 || ourSpeed <= 0) {
      throw new IllegalArgumentException("invalid ZoneMode parameters");
    }
    this.radius = radius;
    this.warningMargin = warningMargin;
    this.anchorDistance = Math.min(anchorDistance, radius);
    this.standoff = standoff;
    this.ourSpeed = ourSpeed;
  }

  public static ZoneMode defaults() {
    return new ZoneMode(4.0, 1.5, 2.0, 1.0, 4.5);
  }

  @Override
  public String name() {
    return "Zone";
  }

  /** Center of the guarded circle. */
  public Vec2 center(FieldConfig field) {
    return field.defendedGoal();
  }

  public double radius() {
    return radius;
  }

  /** Where we wait when no one is threatening the zone. */
  public Vec2 anchor(FieldConfig field) {
    Vec2 goal = field.defendedGoal();
    Vec2 fieldCenter = new Vec2(field.length() / 2.0, field.width() / 2.0);
    Vec2 dir = fieldCenter.minus(goal).normalized();
    if (dir.equals(Vec2.ZERO)) {
      dir = new Vec2(-1.0, 0.0);
    }
    return field.clampInside(goal.plus(dir.times(anchorDistance)));
  }

  @Override
  public Optional<DefenseTarget> compute(DefenseContext ctx) {
    FieldConfig field = ctx.field();
    Vec2 goal = field.defendedGoal();
    Vec2 self = ctx.self().position();

    TrackedRobot threat = null;
    double threatDist = Double.POSITIVE_INFINITY;
    for (TrackedRobot r : ctx.opponents()) {
      double d = r.positionAt(ctx.timeSec()).distanceTo(goal);
      if (d <= radius + warningMargin && d < threatDist) {
        threat = r;
        threatDist = d;
      }
    }

    if (threat == null) {
      Vec2 anchor = anchor(field);
      double heading = anchor.minus(goal).angle(); // face out of the zone
      return Optional.of(new DefenseTarget(anchor, heading, Vec2.ZERO, -1));
    }

    Vec2 oppPos = threat.positionAt(ctx.timeSec());
    OptionalDouble t = Intercept.time(oppPos.minus(self), threat.velocity(), ourSpeed);
    Vec2 meet = t.isPresent() ? oppPos.plus(threat.velocity().times(t.getAsDouble())) : oppPos;

    Vec2 toGoal = goal.minus(meet);
    Vec2 target = meet.plus(toGoal.normalized().times(Math.min(standoff, toGoal.norm())));

    // Stay inside the zone circle.
    Vec2 fromCenter = target.minus(goal);
    if (fromCenter.norm() > radius) {
      target = goal.plus(fromCenter.normalized().times(radius));
    }
    target = field.clampInside(target);

    Vec2 facing = oppPos.minus(target);
    double heading = facing.equals(Vec2.ZERO) ? 0.0 : facing.angle();
    return Optional.of(new DefenseTarget(target, heading, Vec2.ZERO, threat.id()));
  }
}
