package phantom.sim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
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

  // Recent positions of every robot, so simulated drivers can react to a delayed view.
  private record Snapshot(double time, Map<Integer, Vec2> positions) {}

  private static final double HISTORY_SEC = 2.0;
  private final java.util.ArrayDeque<Snapshot> history = new java.util.ArrayDeque<>();

  public SimWorld(FieldConfig field, RobotBody self, List<Opponent> opponents) {
    this.field = field;
    this.self = self;
    this.opponents = new ArrayList<>(opponents);
    this.refereePins = new PinTimer(PinRules.placeholder(), field.robotHalfSize());
    recordHistory();
  }

  private void recordHistory() {
    Map<Integer, Vec2> now = new HashMap<>();
    now.put(self.id(), self.position());
    for (Opponent o : opponents) {
      now.put(o.body().id(), o.body().position());
    }
    history.addLast(new Snapshot(timeSec, now));
    while (history.size() > 1 && history.peekFirst().time() < timeSec - HISTORY_SEC) {
      history.removeFirst();
    }
  }

  /**
   * Every robot's position (keyed by id, including ours) at the latest recorded time at or before
   * {@code t}; the oldest record if {@code t} is earlier than the history.
   */
  public Map<Integer, Vec2> positionsAt(double t) {
    Snapshot best = history.peekFirst();
    for (Snapshot s : history) {
      if (s.time() <= t + 1e-9) {
        best = s;
      } else {
        break;
      }
    }
    return best.positions();
  }

  /** Advance the world by dt seconds with our robot following selfCmd. */
  public void step(ChassisCommand selfCmd, double dt) {
    // Opponents decide based on the world as it is before anyone moves.
    List<ChassisCommand> oppCmds = new ArrayList<>();
    for (Opponent o : opponents) {
      oppCmds.add(o.behavior().decide(timeSec, dt, o.body(), this));
    }

    if (realSelf != null) {
      self.setMeasured(realSelf.pose(), realSelf.velocity(), realSelf.desiredVelocity());
    } else {
      self.step(selfCmd, dt, field);
    }
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
    recordHistory();
    countPins();
  }

  // Phase 7: our robot's measured state for the step in progress; null in the normal sim.
  private record RealSelf(Pose pose, Vec2 velocity, Vec2 desiredVelocity) {}

  private RealSelf realSelf = null;

  /**
   * Phase 7 (ghost opponents on a real robot): advance the opponents by dt seconds while OUR robot
   * is real. Its pose and velocity come from the drivetrain, so nothing here may move it.
   * Everything else (opponent behaviors, opponent-vs-opponent pushing, history, the pin referee)
   * runs exactly as in {@link #step}.
   *
   * <p>Virtual contact with the real robot is one-sided, because a ghost cannot push real metal:
   *
   * <pre>
   *   n        = unit(p_ghost - p_self)
   *   overlap  = 2 * robotHalfSize - |p_ghost - p_self|        (only if > 0)
   *   p_ghost += overlap * n                                   (the ghost gives up ALL of it;
   *                                                             in the sim it gives up w_B)
   *   if (v_self - v_ghost) . n > 0:                           (closing)
   *     v_ghost -= ((v_ghost - v_self) . n) * n                (ghost matches us along n)
   *   p_self, v_self: never changed
   * </pre>
   *
   * So a ghost can be pushed and blocked by the real robot, and nothing here moves the real robot.
   * How the real robot reacts to a ghost pushing on it is up to the robot code (on the robot,
   * frc.robot.defense.VirtualContact makes the assist give way using the same {@link Contact}
   * model).
   *
   * @param selfPose measured pose (field frame)
   * @param selfVelocity measured field velocity (m/s)
   * @param selfDesiredVelocity field velocity the robot is being commanded to drive at (m/s); the
   *     pin referee counts contact only while this points into the opponent
   */
  public void stepWithRealSelf(
      Pose selfPose, Vec2 selfVelocity, Vec2 selfDesiredVelocity, double dt) {
    realSelf = new RealSelf(selfPose, selfVelocity, selfDesiredVelocity);
    try {
      step(ChassisCommand.STOP, dt);
    } finally {
      realSelf = null;
    }
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

        // Phase 7: a real robot (always "a": it is first in the list) is never moved by a ghost.
        boolean realA = realSelf != null && a == self;
        double overlap = minDist - dist;
        double shareB = realA ? 1.0 : Contact.overlapShareB(effortA, effortB);
        if (!realA) {
          a.setPosition(a.position().minus(n.times(overlap * (1.0 - shareB))));
        }
        b.setPosition(b.position().plus(n.times(overlap * shareB)));

        double closing = a.velocity().minus(b.velocity()).dot(n); // > 0 means approaching
        if (closing > 0) {
          double vn = realA ? a.velocity().dot(n) : Contact.contactSpeed(effortA, effortB);
          if (!realA) {
            a.setVelocity(a.velocity().minus(n.times(a.velocity().dot(n) - vn)));
          }
          b.setVelocity(b.velocity().minus(n.times(b.velocity().dot(n) - vn)));
        }
        if (!realA) {
          a.keepInside(field);
        }
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
