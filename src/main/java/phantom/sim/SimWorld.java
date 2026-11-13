package phantom.sim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Vec2;
import phantom.core.rules.PinRules;
import phantom.core.rules.PinTimer;
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

  // Referee view: counts pins from ground truth (no safety margin).
  private final PinTimer refereePins;
  private final Set<Integer> violationCounted = new HashSet<>();
  private int pinViolations = 0;

  public SimWorld(FieldConfig field, RobotBody self, List<Opponent> opponents) {
    this.field = field;
    this.self = self;
    this.opponents = new ArrayList<>(opponents);
    this.refereePins = new PinTimer(PinRules.placeholder(), field.robotHalfSize());
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
    countPins();
  }

  /**
   * One violation per pin episode whose contact time reaches the limit. Like a referee, only
   * counts contact while we are driving into that opponent (effort > 0.1), not while we are being
   * pushed or are backing away.
   */
  private void countPins() {
    Map<Integer, Vec2> positions = new HashMap<>();
    Map<Integer, Vec2> towardOpp = new HashMap<>();
    for (Opponent o : opponents) {
      positions.put(o.body().id(), o.body().position());
      towardOpp.put(o.body().id(), o.body().position().minus(self.position()).normalized());
    }
    refereePins.update(timeSec, self.position(), positions,
        id -> Contact.effort(self.desiredVelocity(), towardOpp.get(id), self.maxSpeed()) > 0.1);
    double limit = refereePins.rules().limitSeconds();
    for (int id : positions.keySet()) {
      if (!refereePins.episodeOpen(id)) {
        violationCounted.remove(id);
      } else if (refereePins.contactSeconds(id) >= limit && violationCounted.add(id)) {
        pinViolations++;
      }
    }
  }

  /**
   * Treat robots as circles of radius robotHalfSize. Overlapping pairs are separated and given a
   * shared velocity along the contact normal according to the pushing-match model in {@link
   * Contact}: whoever is driving harder into the other wins; equal effort stalls.
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
        double effortA = Contact.effort(a.desiredVelocity(), n, a.maxSpeed());
        double effortB = Contact.effort(b.desiredVelocity(), n.times(-1.0), b.maxSpeed());

        double overlap = minDist - dist;
        double shareB = Contact.overlapShareB(effortA, effortB);
        a.setPosition(a.position().minus(n.times(overlap * (1.0 - shareB))));
        b.setPosition(b.position().plus(n.times(overlap * shareB)));

        double closing = a.velocity().minus(b.velocity()).dot(n); // > 0 means approaching
        if (closing > 0) {
          double vn = Contact.contactSpeed(effortA, effortB);
          a.setVelocity(a.velocity().minus(n.times(a.velocity().dot(n) - vn)));
          b.setVelocity(b.velocity().minus(n.times(b.velocity().dot(n) - vn)));
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

  /** Pin episodes that reached the limit, counting only time we spent driving into them. */
  public int pinViolations() {
    return pinViolations;
  }

  /** Total seconds our robot has been in contact with any opponent. */
  public double selfContactSeconds() {
    return selfContactSeconds;
  }
}
