package phantom.core.modes;

import java.util.Optional;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Line mirror: stay on a fixed wall line in front of the goal and slide along it to match the
 * opponent. Simplest mode and the most robust to bad tracking, since only one coordinate matters.
 *
 * <pre>
 *   d  = unit(fieldCenter - goal)            (direction out from the goal)
 *   L0 = goal + offset * d                   (a point on the line)
 *   t  = (-d.y, d.x)                         (line direction, perpendicular to d)
 *   p* = L0 + ((p_hat - L0) . t) * t         (opponent projected onto the line)
 *   v_ff = (v_opp . t) * t                   (only the along-line part of their motion)
 * </pre>
 */
public final class LineMirrorMode implements DefenseMode {
  private final double offset;
  private final double leadSeconds;

  /**
   * @param offset distance of the line from the goal, toward field center (m)
   * @param leadSeconds how far ahead to predict the opponent (s)
   */
  public LineMirrorMode(double offset, double leadSeconds) {
    if (offset < 0 || leadSeconds < 0) {
      throw new IllegalArgumentException("offset and leadSeconds must be >= 0");
    }
    this.offset = offset;
    this.leadSeconds = leadSeconds;
  }

  public static LineMirrorMode defaults() {
    return new LineMirrorMode(3.0, 0.15);
  }

  @Override
  public String name() {
    return "Line mirror";
  }

  /** Unit vector pointing from the goal toward field center. */
  public Vec2 outward(FieldConfig field) {
    Vec2 center = new Vec2(field.length() / 2.0, field.width() / 2.0);
    Vec2 d = center.minus(field.defendedGoal()).normalized();
    return d.equals(Vec2.ZERO) ? new Vec2(-1.0, 0.0) : d;
  }

  /** A point on the line. */
  public Vec2 linePoint(FieldConfig field) {
    return field.defendedGoal().plus(outward(field).times(offset));
  }

  /** Unit direction along the line. */
  public Vec2 lineDirection(FieldConfig field) {
    Vec2 d = outward(field);
    return new Vec2(-d.y(), d.x());
  }

  @Override
  public Optional<DefenseTarget> compute(DefenseContext ctx) {
    Optional<TrackedRobot> focus = ctx.focusOpponent();
    if (focus.isEmpty()) {
      return Optional.empty();
    }
    TrackedRobot opp = focus.get();
    FieldConfig field = ctx.field();
    Vec2 l0 = linePoint(field);
    Vec2 t = lineDirection(field);

    Vec2 predicted = opp.positionAt(ctx.timeSec() + leadSeconds);
    Vec2 target = field.clampInside(l0.plus(t.times(predicted.minus(l0).dot(t))));
    Vec2 ff = t.times(opp.velocity().dot(t));

    Vec2 facing = predicted.minus(target);
    double heading = facing.equals(Vec2.ZERO) ? outward(field).angle() : facing.angle();
    return Optional.of(new DefenseTarget(target, heading, ff, opp.id()));
  }
}
