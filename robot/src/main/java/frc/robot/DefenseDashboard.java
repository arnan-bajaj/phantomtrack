package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.defense.GhostSettings;
import frc.robot.defense.SafetyConfig;
import phantom.sim.Scenarios;

/**
 * Dashboard settings under SmartDashboard/PhantomTrack/Settings (Elastic, Glass, Shuffleboard or
 * AdvantageScope can edit them). Every boot starts from the first-test values: nothing typed in a
 * previous session survives a reboot.
 *
 * <p>Changes are only requests: PhantomTrack applies them on the next loop with the assist
 * trigger released. Out-of-range numbers are pulled back into range (SafetyConfig.withTunables).
 */
public final class DefenseDashboard {
  private static final String P = "PhantomTrack/Settings/";

  /** What the dashboard asks for this loop. */
  public record Settings(
      SafetyConfig safety, GhostSettings ghosts, double driverMaxSpeed, boolean resetGhosts) {}

  private final SafetyConfig base;
  private final SendableChooser<Integer> count = new SendableChooser<>();
  private final SendableChooser<Scenarios.OpponentKind> kind = new SendableChooser<>();
  private final SendableChooser<Scenarios.Layout> layout = new SendableChooser<>();

  public DefenseDashboard(SafetyConfig base, GhostSettings ghosts) {
    this.base = base;
    for (int n = 0; n <= GhostSettings.MAX_COUNT; n++) {
      if (n == ghosts.count()) {
        count.setDefaultOption(Integer.toString(n), n);
      } else {
        count.addOption(Integer.toString(n), n);
      }
    }
    for (Scenarios.OpponentKind k : Scenarios.OpponentKind.values()) {
      String name = k == Scenarios.OpponentKind.CYCLER ? "Cycler (drives through)" : "Evader (dodges)";
      if (k == ghosts.kind()) {
        kind.setDefaultOption(name, k);
      } else {
        kind.addOption(name, k);
      }
    }
    for (Scenarios.Layout l : Scenarios.Layout.values()) {
      if (l == ghosts.layout()) {
        layout.setDefaultOption(l.field().name(), l);
      } else {
        layout.addOption(l.field().name(), l);
      }
    }
    SmartDashboard.putData(P + "Ghost count", count);
    SmartDashboard.putData(P + "Ghost type", kind);
    SmartDashboard.putData(P + "Field layout", layout);
    SmartDashboard.putNumber(P + "Ghost max speed (m/s)", ghosts.maxSpeed());
    SmartDashboard.putNumber(P + "Assist speed cap (m/s)", base.speedCap());
    SmartDashboard.putNumber(P + "Assist accel limit (m/s^2)", base.accelLimit());
    SmartDashboard.putNumber(P + "Driver max speed (m/s)", Constants.DRIVER_MAX_SPEED);
    SmartDashboard.putNumber(P + "Area min X (m)", base.areaMinX());
    SmartDashboard.putNumber(P + "Area min Y (m)", base.areaMinY());
    SmartDashboard.putNumber(P + "Area max X (m)", base.areaMaxX());
    SmartDashboard.putNumber(P + "Area max Y (m)", base.areaMaxY());
    SmartDashboard.putNumber(P + "Fence margin (m)", base.fenceMargin());
    SmartDashboard.putBoolean(P + "Reset ghosts", false);
  }

  /** Read every setting. Call once per loop. */
  public Settings read() {
    SafetyConfig safety =
        base.withTunables(
            SmartDashboard.getNumber(P + "Assist speed cap (m/s)", base.speedCap()),
            SmartDashboard.getNumber(P + "Assist accel limit (m/s^2)", base.accelLimit()),
            SmartDashboard.getNumber(P + "Area min X (m)", base.areaMinX()),
            SmartDashboard.getNumber(P + "Area min Y (m)", base.areaMinY()),
            SmartDashboard.getNumber(P + "Area max X (m)", base.areaMaxX()),
            SmartDashboard.getNumber(P + "Area max Y (m)", base.areaMaxY()),
            SmartDashboard.getNumber(P + "Fence margin (m)", base.fenceMargin()));
    Integer n = count.getSelected();
    Scenarios.OpponentKind k = kind.getSelected();
    Scenarios.Layout l = layout.getSelected();
    GhostSettings ghosts =
        new GhostSettings(
            l == null ? Scenarios.Layout.OPEN : l,
            k == null ? Scenarios.OpponentKind.CYCLER : k,
            n == null ? 1 : n,
            SmartDashboard.getNumber(P + "Ghost max speed (m/s)", 1.0));
    double driverMax = SmartDashboard.getNumber(P + "Driver max speed (m/s)", Constants.DRIVER_MAX_SPEED);
    driverMax =
        Double.isFinite(driverMax)
            ? Math.max(0.0, Math.min(SafetyConfig.MAX_SPEED_CAP, driverMax))
            : Constants.DRIVER_MAX_SPEED;
    boolean reset = SmartDashboard.getBoolean(P + "Reset ghosts", false);
    if (reset) {
      SmartDashboard.putBoolean(P + "Reset ghosts", false);
    }
    return new Settings(safety, ghosts, driverMax, reset);
  }
}
