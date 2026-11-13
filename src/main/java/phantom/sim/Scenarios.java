package phantom.sim;

import java.util.ArrayList;
import java.util.List;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/** Ready-made starting setups for the sim. */
public final class Scenarios {
  private Scenarios() {}

  /** Which kind of opponent bot to use. */
  public enum OpponentKind {
    /** Drives straight through defenders. */
    CYCLER,
    /** Steers around defenders. */
    EVADER
  }

  /** Our robot near the defended goal; three Cycler opponents, one per pickup. */
  public static SimWorld threeCyclers(FieldConfig field) {
    return three(field, OpponentKind.CYCLER);
  }

  /** Where our robot starts when defending: in front of the defended goal. */
  public static final Pose DEFENDER_START = new Pose(11.0, 4.035, Math.PI);

  /** Where our robot parks for the no-defense baseline: a corner far from every lane. */
  public static Pose parkedOutOfTheWay(FieldConfig field) {
    double m = field.robotHalfSize() + 0.05;
    return new Pose(field.length() - m, field.width() - m, Math.PI);
  }

  /** Our robot near the defended goal; three opponents of the given kind, one per pickup. */
  public static SimWorld three(FieldConfig field, OpponentKind kind) {
    return three(field, kind, DEFENDER_START);
  }

  /** Three opponents of the given kind, one per pickup; our robot starts at selfStart. */
  public static SimWorld three(FieldConfig field, OpponentKind kind, Pose selfStart) {
    RobotBody self = RobotBody.typical(100, selfStart);
    List<SimWorld.Opponent> opps = new ArrayList<>();
    int id = 0;
    for (Vec2 pickup : field.opponentPickups()) {
      RobotBody body = RobotBody.typical(id, new Pose(pickup, 0.0));
      // Scores from up to 1.5 m away (a shot), waits 0.5 s at each end.
      CyclerBehavior behavior =
          kind == OpponentKind.EVADER
              ? CyclerBehavior.evader(pickup, field.defendedGoal(), 1.5, 0.5)
              : new CyclerBehavior(pickup, field.defendedGoal(), 1.5, 0.5);
      opps.add(new SimWorld.Opponent(body, behavior));
      id++;
    }
    return new SimWorld(field, self, opps);
  }
}
