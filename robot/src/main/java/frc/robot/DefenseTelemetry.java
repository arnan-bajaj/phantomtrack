package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.defense.Geofence;
import frc.robot.defense.PhantomTrack;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.modes.DefenseTarget;

/**
 * NetworkTables output under /PhantomTrack, in AdvantageScope-friendly struct types. Everything
 * with a position is in the REAL field frame, so ghosts show up where they are relative to the
 * real robot (robot/advantagescope-layout.json shows them on a 2D field). DataLogManager records
 * all of it to the robot's .wpilog.
 *
 * <p>In simulation it also prints one status line per second to the console.
 */
public final class DefenseTelemetry {
  private static final String T = "/PhantomTrack/";
  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

  private final StructPublisher<Pose2d> robotPose =
      nt.getStructTopic(T + "RobotPose", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> bluePose =
      nt.getStructTopic(T + "BlueFrame/RobotPose", Pose2d.struct).publish();
  private final StructArrayPublisher<Pose2d> ghosts =
      nt.getStructArrayTopic(T + "Ghosts", Pose2d.struct).publish();
  private final StructArrayPublisher<Pose2d> target =
      nt.getStructArrayTopic(T + "Target", Pose2d.struct).publish();
  private final StructArrayPublisher<Translation2d> fence =
      nt.getStructArrayTopic(T + "Fence", Translation2d.struct).publish();
  private final StructArrayPublisher<Translation2d> goal =
      nt.getStructArrayTopic(T + "Goal", Translation2d.struct).publish();
  private final StructPublisher<ChassisSpeeds> commanded =
      nt.getStructTopic(T + "CommandedSpeeds", ChassisSpeeds.struct).publish();
  private final StringPublisher status = nt.getStringTopic(T + "Status").publish();
  private final StringPublisher decision = nt.getStringTopic(T + "Decision").publish();
  private final StringPublisher mode = nt.getStringTopic(T + "Mode").publish();
  private final StringPublisher focus = nt.getStringTopic(T + "Focus").publish();
  private final StringPublisher alliance = nt.getStringTopic(T + "Alliance").publish();
  private final StringPublisher poseFault = nt.getStringTopic(T + "PoseFault").publish();
  private final BooleanPublisher assistDriving = nt.getBooleanTopic(T + "AssistDriving").publish();
  private final BooleanPublisher hold = nt.getBooleanTopic(T + "Hold").publish();
  private final BooleanPublisher poseHealthy = nt.getBooleanTopic(T + "PoseHealthy").publish();
  private final BooleanPublisher pinWarning = nt.getBooleanTopic(T + "PinWarning").publish();
  private final BooleanPublisher settingsPending =
      nt.getBooleanTopic(T + "SettingsPending").publish();
  private final DoubleArrayPublisher pinSeconds =
      nt.getDoubleArrayTopic(T + "PinSeconds").publish();
  private final DoublePublisher maxPinSeconds = nt.getDoubleTopic(T + "MaxPinSeconds").publish();
  private final DoublePublisher speedCap = nt.getDoubleTopic(T + "SpeedCap").publish();
  private final DoublePublisher targetDistance = nt.getDoubleTopic(T + "TargetDistance").publish();
  private final DoublePublisher commandedSpeed = nt.getDoubleTopic(T + "CommandedSpeed").publish();
  private final IntegerPublisher guarded = nt.getIntegerTopic(T + "GuardedGhost").publish();
  private final IntegerPublisher pins = nt.getIntegerTopic(T + "PinViolations").publish();

  private double lastPrint = Double.NEGATIVE_INFINITY;

  /**
   * @param realPose the drivetrain's pose this loop (may be null)
   * @param realOut what was sent to the drivetrain this loop (real frame)
   */
  public void publish(
      double now,
      PhantomTrack pt,
      AllianceFrame frame,
      boolean red,
      Pose2d realPose,
      ChassisSpeeds realOut) {
    if (realPose != null) {
      robotPose.set(realPose);
      bluePose.set(Convert.toWpi(frame.toBlue(realPose, red)));
    }
    List<Pose2d> g = new ArrayList<>();
    for (Pose p : pt.ghostPoses()) {
      g.add(frame.poseToReal(p, red));
    }
    ghosts.set(g.toArray(new Pose2d[0]));
    DefenseTarget t = pt.target().orElse(null);
    target.set(
        t == null
            ? new Pose2d[0]
            : new Pose2d[] {frame.poseToReal(new Pose(t.position(), t.heading()), red)});
    Geofence f = pt.fence();
    if (f != null) {
      List<Vec2> c = f.fenceCorners();
      fence.set(
          new Translation2d[] {
            frame.pointToReal(c.get(0), red),
            frame.pointToReal(c.get(1), red),
            frame.pointToReal(c.get(2), red),
            frame.pointToReal(c.get(3), red),
            frame.pointToReal(c.get(0), red)
          });
    } else {
      fence.set(new Translation2d[0]);
    }
    FieldConfig field = pt.field();
    goal.set(
        field == null
            ? new Translation2d[0]
            : new Translation2d[] {frame.pointToReal(field.defendedGoal(), red)});
    commanded.set(realOut);
    status.set(pt.status());
    decision.set(pt.decision().toString());
    mode.set(pt.modeName());
    focus.set(pt.focus() < 0 ? "auto" : "#" + pt.focus());
    alliance.set(red ? "Red" : "Blue");
    poseFault.set(pt.currentFault().toString());
    assistDriving.set(pt.decision() == frc.robot.defense.SafetyGate.Decision.ASSIST);
    hold.set(pt.hold());
    poseHealthy.set(pt.poseHealthy());
    pinWarning.set(pt.pinWarning());
    settingsPending.set(pt.settingsPending());
    pinSeconds.set(pt.pinSeconds());
    maxPinSeconds.set(pt.maxPinSeconds());
    speedCap.set(pt.safety().speedCap());
    double dist =
        t == null || realPose == null
            ? Double.NaN
            : frame.toBlue(realPose, red).position().distanceTo(t.position());
    targetDistance.set(dist);
    commandedSpeed.set(Math.hypot(realOut.vxMetersPerSecond, realOut.vyMetersPerSecond));
    guarded.set(pt.guardedId());
    pins.set(pt.pinViolations());

    if (RobotBase.isSimulation() && now - lastPrint >= 1.0) {
      lastPrint = now;
      System.out.println(statusLine(now, pt, red, realPose, g, dist, realOut));
    }
  }

  private static String statusLine(
      double now,
      PhantomTrack pt,
      boolean red,
      Pose2d realPose,
      List<Pose2d> realGhosts,
      double dist,
      ChassisSpeeds out) {
    StringBuilder sb = new StringBuilder();
    sb.append(
        String.format(
            Locale.ROOT,
            "[PT] t=%.1f %s %s | %s | mode=%s focus=%s guarded=%d",
            now,
            red ? "RED" : "BLUE",
            realPose == null
                ? "pose=null"
                : String.format(
                    Locale.ROOT,
                    "pose=(%.2f,%.2f,%.0fdeg)",
                    realPose.getX(),
                    realPose.getY(),
                    realPose.getRotation().getDegrees()),
            pt.status(),
            pt.modeName(),
            pt.focus() < 0 ? "auto" : "#" + pt.focus(),
            pt.guardedId()));
    pt.target()
        .ifPresent(
            t ->
                sb.append(
                    String.format(
                        Locale.ROOT, " target(blue)=(%.2f,%.2f)", t.position().x(), t.position().y())));
    sb.append(String.format(Locale.ROOT, " dist=%.2f", dist));
    sb.append(
        String.format(
            Locale.ROOT, " cmd=%.2f", Math.hypot(out.vxMetersPerSecond, out.vyMetersPerSecond)));
    for (int i = 0; i < realGhosts.size(); i++) {
      sb.append(
          String.format(
              Locale.ROOT, " g%d=(%.2f,%.2f)", i, realGhosts.get(i).getX(), realGhosts.get(i).getY()));
    }
    sb.append(String.format(Locale.ROOT, " pin=%.2f", pt.maxPinSeconds()));
    if (pt.pinWarning()) {
      sb.append(" RUMBLE");
    }
    return sb.toString();
  }
}
