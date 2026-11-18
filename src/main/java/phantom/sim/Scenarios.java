package phantom.sim;

import java.util.ArrayList;
import java.util.List;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;

/** Ready-made starting setups for the sim. */
public final class Scenarios {
  private Scenarios() {}

  /** Which field to play on. */
  public enum Layout {
    OPEN,
    CHOKE,
    TWO_LANES;

    public FieldConfig field() {
      return switch (this) {
        case OPEN -> FieldConfig.placeholder();
        case CHOKE -> FieldConfig.choke();
        case TWO_LANES -> FieldConfig.twoLanes();
      };
    }

    public Layout next() {
      return values()[(ordinal() + 1) % values().length];
    }
  }

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
    return three(field, kind, selfStart, false);
  }

  /** Body ids of the two alliance partners. */
  public static final int FIRST_PARTNER_ID = 200;

  /**
   * Same, optionally with two alliance partners on our side.
   *
   * <p>Partner layout (made up, like the rest of the placeholder field): our alliance mirrors the
   * opponents' layout through the field's center line, so partners load where the first two
   * opponent pickups would be on the far side and score at the mirror image of the goal we defend:
   *
   * <pre>
   *   mirror(p) = (fieldLength - p.x, p.y)
   *   partner i:  pickup = mirror(opponentPickups[i]),  goal = mirror(defendedGoal)
   * </pre>
   *
   * So partners cross the whole field through our defensive area and through the opponents'
   * lanes, as real partners do in games where you load at the far end. They always drive like
   * Evaders (steer around robots, human reaction delay), even when the opponents are Cyclers: a
   * teammate does not drive through its own defender.
   */
  public static SimWorld three(
      FieldConfig field, OpponentKind kind, Pose selfStart, boolean withPartners) {
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
    List<SimWorld.Opponent> partners = new ArrayList<>();
    if (withPartners) {
      Vec2 ourGoal = mirror(field, field.defendedGoal());
      for (int i = 0; i < 2 && i < field.opponentPickups().size(); i++) {
        Vec2 pickup = mirror(field, field.opponentPickups().get(i));
        RobotBody body = RobotBody.typical(FIRST_PARTNER_ID + i, new Pose(pickup, Math.PI));
        partners.add(new SimWorld.Opponent(body, CyclerBehavior.evader(pickup, ourGoal, 1.5, 0.5)));
      }
    }
    return new SimWorld(field, self, opps, partners);
  }

  /** Reflection through the field's center line x = length / 2. */
  public static Vec2 mirror(FieldConfig field, Vec2 p) {
    return new Vec2(field.length() - p.x(), p.y());
  }
}
