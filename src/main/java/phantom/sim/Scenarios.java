package phantom.sim;

import java.util.ArrayList;
import java.util.List;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/** Ready-made starting setups for the sim. */
public final class Scenarios {
  private Scenarios() {}

  /** Our robot near the defended goal; three Cycler opponents, one per pickup. */
  public static SimWorld threeCyclers(FieldConfig field) {
    RobotBody self = RobotBody.typical(100, new Pose(11.0, 4.035, Math.PI));
    List<SimWorld.Opponent> opps = new ArrayList<>();
    int id = 0;
    for (Vec2 pickup : field.opponentPickups()) {
      RobotBody body = RobotBody.typical(id, new Pose(pickup, 0.0));
      // Scores from up to 1.5 m away (a shot), waits 0.5 s at each end.
      opps.add(new SimWorld.Opponent(body, new CyclerBehavior(pickup, field.defendedGoal(), 1.5, 0.5)));
      id++;
    }
    return new SimWorld(field, self, opps);
  }
}
