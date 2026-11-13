package phantom.sim;

import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import javax.imageio.ImageIO;
import phantom.core.control.ChassisCommand;
import phantom.core.field.FieldConfig;
import phantom.sim.ui.FieldRenderer;
import phantom.sim.ui.SimWindow;

/**
 * Entry point.
 *
 * <pre>
 *   ./gradlew run                                   interactive window
 *   ./gradlew run --args="--headless"               180 s comparison of every setup vs both opponent kinds
 *   ./gradlew run --args="--headless 600"           same, 600 s
 * </pre>
 */
public final class SimMain {
  /** Opponent the man-to-man modes guard in headless runs. Zone ignores focus. */
  static final int GUARDED_ID = 0;

  private SimMain() {}

  public static void main(String[] args) throws IOException {
    boolean headless = args.length > 0 && args[0].equals("--headless");
    if (!headless && GraphicsEnvironment.isHeadless()) {
      System.out.println("No display found, running headless instead.");
      headless = true;
    }
    if (!headless) {
      SimWindow.open();
      return;
    }
    double seconds = args.length > 1 ? Double.parseDouble(args[1]) : 180.0;
    runHeadless(seconds);
  }

  /** One configuration to compare. modeIndex -1 = no defense (robot parked in a corner). */
  record Setup(String label, int modeIndex, boolean hold, boolean guard) {}

  /** The standard list of setups compared in headless runs. */
  static List<Setup> standardSetups() {
    List<Setup> setups = new ArrayList<>();
    setups.add(new Setup("No defense", -1, false, true));
    String[] names = {"Shadow", "Interpose", "Zone", "Line mirror"};
    for (int m = 0; m < names.length; m++) {
      setups.add(new Setup(names[m], m, false, true));
      setups.add(new Setup(names[m] + " + hold", m, true, true));
    }
    for (int m = 0; m < names.length; m++) {
      setups.add(new Setup(names[m] + " + hold, NO pin guard", m, true, false));
    }
    return setups;
  }

  /** Builds and runs one setup for the given time. */
  static SimSession run(Setup setup, Scenarios.OpponentKind kind, double seconds, File snapshot)
      throws IOException {
    FieldConfig field = FieldConfig.placeholder();
    SimSession s =
        setup.modeIndex() < 0
            ? new SimSession(Scenarios.three(field, kind, Scenarios.parkedOutOfTheWay(field)))
            : SimSession.standard(kind);
    if (setup.modeIndex() >= 0) {
      s.assist().setModeIndex(setup.modeIndex());
      s.assist().setHoldEnabled(setup.hold());
      s.assist().setFocusOpponentId(GUARDED_ID); // man-to-man modes lock onto one scorer
    }
    s.assist().setRulesGuardEnabled(setup.guard());
    boolean engaged = setup.modeIndex() >= 0;
    int steps = (int) Math.round(seconds / SimSession.DT);
    for (int i = 0; i < steps; i++) {
      s.step(ChassisCommand.STOP, engaged);
      if (snapshot != null && i == steps / 2) {
        ImageIO.write(render(s), "png", snapshot);
      }
    }
    return s;
  }

  static void runHeadless(double seconds) throws IOException {
    for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
      runComparison(kind, seconds);
      System.out.println();
    }
  }

  static void runComparison(Scenarios.OpponentKind kind, double seconds) throws IOException {
    File dir = new File("snapshots");
    dir.mkdirs();
    String kindName = kind.name().toLowerCase();

    System.out.printf("=== %.0f s, 3 %s opponents (guarding #%d) ===%n", seconds,
        kind == Scenarios.OpponentKind.EVADER ? "Evader (dodge defenders)" : "Cycler (push through)",
        GUARDED_ID);
    System.out.printf("%-34s %12s %8s %8s %9s %6s %10s%n",
        "Setup", "Guarded cyc", "Slowed", "Total", "Contact", "Pins", "Back-offs");

    double base = Double.NaN;
    for (Setup setup : standardSetups()) {
      String file = kindName + "-" + setup.label().toLowerCase().replaceAll("[^a-z0-9]+", "-") + ".png";
      boolean snap = setup.guard() && setup.modeIndex() >= 0;
      SimSession s = run(setup, kind, seconds, snap ? new File(dir, file) : null);

      double cycle = s.world().opponents().get(GUARDED_ID).behavior().averageCycleSeconds();
      if (setup.modeIndex() < 0) {
        base = cycle;
      }
      int total = 0;
      for (SimWorld.Opponent o : s.world().opponents()) {
        total += o.behavior().cyclesCompleted();
      }
      String slowed =
          setup.modeIndex() < 0 || Double.isNaN(cycle) || Double.isNaN(base)
              ? "-"
              : String.format("%+.0f%%", 100 * (cycle - base) / base);
      System.out.printf("%-34s %10.2f s %8s %8d %7.1f s %6d %10d%n",
          setup.label(), cycle, slowed, total, s.world().selfContactSeconds(),
          s.world().pinViolations(), s.assist().backoffsTriggered());
    }
    System.out.println("Snapshots: " + dir.getPath() + "/" + kindName + "-*.png");
  }

  static BufferedImage render(SimSession s) {
    BufferedImage img = new BufferedImage(1320, 780, BufferedImage.TYPE_INT_RGB);
    Graphics2D g = img.createGraphics();
    new FieldRenderer().paint(g, img.getWidth(), img.getHeight(), s, false);
    g.dispose();
    return img;
  }
}
