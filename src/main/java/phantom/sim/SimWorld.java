package phantom.sim;

import java.util.ArrayList;
import java.util.List;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Vec2;
import phantom.core.tracking.TrackedRobot;

/**
 * Ground-truth world: our robot plus opponent bots. Knows every pose exactly. What the defense
 * code is allowed to see goes through an OpponentTracker, never through this class directly.
 */
public final class SimWorld {
  /** One opponent: its body and the behavior driving it. */
  public record Opponent(RobotBody body, OpponentBehavior behavior) {}

  private final FieldConfig field;
  private final RobotBody self;
  private final List<Opponent> opponents;
  private double timeSec = 0.0;
  private double selfContactSeconds = 0.0;

  public SimWorld(FieldConfig field, RobotBody self, List<Opponent> opponents) {
    this.field = field;
    this.self = self;
    this.opponents = new ArrayList<>(opponents);
  }

  /** Advance the world by dt seconds with our robot following selfCmd. */
  public void step(ChassisCommand selfCmd, double dt) {
    // Opponents decide based on the world as it is before anyone moves.
    List<ChassisCommand> oppCmds = new ArrayList<>();
    for (Opponent o : opponents) {
      oppCmds.add(o.behavior().decide(timeSec, dt, o.body(), this));
    }

    self.step(selfCmd, dt, field);
    for (int i = 0; i < opponents.size(); i++) {
      opponents.get(i).body().step(oppCmds.get(i), dt, field);
    }

    boolean selfTouching = false;
    for (int iter = 0; iter < 3; iter++) {
      selfTouching |= resolveCollisions();
    }
    if (selfTouching) {
      selfContactSeconds += dt;
    }
    timeSec += dt;
  }

  /**
   * Treat robots as circles of radius robotHalfSize. Push overlapping pairs apart equally and
   * remove the part of their velocity that drives them into each other.
   *
   * @return true if our robot touched any opponent
   */
  private boolean resolveCollisions() {
    List<RobotBody> all = allBodies();
    double minDist = 2.0 * field.robotHalfSize();
    boolean selfTouching = false;
    for (int i = 0; i < all.size(); i++) {
      for (int j = i + 1; j < all.size(); j++) {
        RobotBody a = all.get(i);
        RobotBody b = all.get(j);
        Vec2 delta = b.position().minus(a.position());
        double dist = delta.norm();
        if (dist >= minDist) {
          continue;
        }
        if (a == self || b == self) {
          selfTouching = true;
        }
        Vec2 n = dist < 1e-9 ? new Vec2(1.0, 0.0) : delta.times(1.0 / dist);
        double overlap = minDist - dist;
        a.setPosition(a.position().minus(n.times(overlap / 2.0)));
        b.setPosition(b.position().plus(n.times(overlap / 2.0)));

        double closing = a.velocity().minus(b.velocity()).dot(n); // > 0 means approaching
        if (closing > 0) {
          a.setVelocity(a.velocity().minus(n.times(closing / 2.0)));
          b.setVelocity(b.velocity().plus(n.times(closing / 2.0)));
        }
        a.keepInside(field);
        b.keepInside(field);
      }
    }
    return selfTouching;
  }

  /** Exact opponent states, as a perfect sensor would report them. */
  public List<TrackedRobot> truth() {
    List<TrackedRobot> out = new ArrayList<>();
    for (Opponent o : opponents) {
      RobotBody b = o.body();
      out.add(new TrackedRobot(b.id(), b.position(), b.velocity(), timeSec, 1.0));
    }
    return out;
  }

  private List<RobotBody> allBodies() {
    List<RobotBody> all = new ArrayList<>();
    all.add(self);
    for (Opponent o : opponents) {
      all.add(o.body());
    }
    return all;
  }

  public FieldConfig field() {
    return field;
  }

  public RobotBody self() {
    return self;
  }

  public List<Opponent> opponents() {
    return opponents;
  }

  public double timeSec() {
    return timeSec;
  }

  /** Total seconds our robot has been in contact with any opponent. */
  public double selfContactSeconds() {
    return selfContactSeconds;
  }
}
