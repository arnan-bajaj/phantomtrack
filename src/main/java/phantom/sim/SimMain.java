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
 *   ./gradlew run --args="--batch"                  every combination x seeds -> results/batch.csv
 *   ./gradlew run --args="--batch 180 10"           same, 180 s per run, 10 seeds
 * </pre>
 */
public final class SimMain {
  /** Opponent the man-to-man modes guard in headless runs. Zone ignores focus. */
  static final int GUARDED_ID = 0;

  private SimMain() {}

  public static void main(String[] args) throws IOException {
    if (args.length > 0 && args[0].equals("--batch")) {
      double seconds = args.length > 1 ? Double.parseDouble(args[1]) : 180.0;
      int seeds = args.length > 2 ? Integer.parseInt(args[2]) : 3;
      BatchRunner.run(seconds, seeds, new File("results", "batch.csv"));
      return;
    }
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

  /**
   * One configuration to compare. modeIndex -1 = no defense (robot parked in a corner). focus -1 =
   * auto (threatSelector picks threat-based selection, otherwise nearest).
   */
  record Setup(String label, int modeIndex, boolean hold, boolean guard, SimSession.Sensor sensor,
      long seed, int focus, boolean threatSelector, Scenarios.Layout layout) {
    Setup(String label, int modeIndex, boolean hold, boolean guard, SimSession.Sensor sensor,
        long seed, int focus, boolean threatSelector) {
      this(label, modeIndex, hold, guard, sensor, seed, focus, threatSelector, Scenarios.Layout.OPEN);
    }

    Setup(String label, int modeIndex, boolean hold, boolean guard) {
      this(label, modeIndex, hold, guard, SimSession.Sensor.PERFECT, 1L);
    }

    Setup(String label, int modeIndex, boolean hold, boolean guard, SimSession.Sensor sensor,
        long seed) {
      this(label, modeIndex, hold, guard, sensor, seed, GUARDED_ID, true);
    }

    Setup withSensor(SimSession.Sensor s, long newSeed) {
      return new Setup(label, modeIndex, hold, guard, s, newSeed, focus, threatSelector, layout);
    }

    Setup withLayout(Scenarios.Layout l) {
      return new Setup(label, modeIndex, hold, guard, sensor, seed, focus, threatSelector, l);
    }
  }

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
    FieldConfig field = setup.layout().field();
    SimSession s =
        setup.modeIndex() < 0
            ? new SimSession(Scenarios.three(field, kind, Scenarios.parkedOutOfTheWay(field)))
            : SimSession.standard(kind, setup.sensor(), setup.seed(), setup.layout());
    if (setup.modeIndex() >= 0) {
      s.assist().setModeIndex(setup.modeIndex());
      s.assist().setHoldEnabled(setup.hold());
      s.assist().setFocusOpponentId(setup.focus());
      s.assist().setThreatSelectorEnabled(setup.threatSelector());
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
    runTrackingComparison(seconds);
    System.out.println();
    runAutoFocusComparison(seconds);
    System.out.println();
    runLayoutComparison(seconds);
  }

  /** Does the ranking survive walls? Open field vs one choke vs two lanes. */
  static void runLayoutComparison(double seconds) throws IOException {
    System.out.printf("=== Field layouts: %.0f s, perfect tracking, guarded opp slowed (all-opp slowed) ===%n",
        seconds);
    List<Setup> setups = List.of(
        new Setup("Shadow + hold, locked #0", 0, true, true),
        new Setup("Interpose + hold, locked #0", 1, true, true),
        new Setup("Zone + hold", 2, true, true),
        new Setup("Line mirror + hold, locked #0", 3, true, true),
        new Setup("Line mirror + hold, auto=threat", 3, true, true, SimSession.Sensor.PERFECT, 1, -1, true));
    StringBuilder head = new StringBuilder(String.format("%-8s %-33s", "Opps", "Setup"));
    for (Scenarios.Layout l : Scenarios.Layout.values()) {
      head.append(String.format("%20s", l.field().name()));
    }
    System.out.println(head.append(String.format("%7s", "Pins")));
    for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
      double[] baseCycle = new double[Scenarios.Layout.values().length];
      int[] baseTotal = new int[baseCycle.length];
      for (Scenarios.Layout l : Scenarios.Layout.values()) {
        SimSession b = run(new Setup("none", -1, false, true).withLayout(l), kind, seconds, null);
        baseCycle[l.ordinal()] = b.world().opponents().get(GUARDED_ID).behavior().averageCycleSeconds();
        baseTotal[l.ordinal()] = totalCycles(b);
      }
      for (Setup setup : setups) {
        StringBuilder row = new StringBuilder(String.format("%-8s %-33s", kindLabel(kind), setup.label()));
        int pins = 0;
        for (Scenarios.Layout l : Scenarios.Layout.values()) {
          SimSession s = run(setup.withLayout(l), kind, seconds, null);
          double c = s.world().opponents().get(GUARDED_ID).behavior().averageCycleSeconds();
          int total = totalCycles(s);
          String guarded = Double.isNaN(c) ? "jam" : String.format("%+.0f%%", 100 * (c - baseCycle[l.ordinal()]) / baseCycle[l.ordinal()]);
          String all = String.format("%+.0f%%", 100.0 * (baseTotal[l.ordinal()] - total) / total);
          row.append(String.format("%20s", guarded + " (" + all + ")"));
          pins += s.world().pinViolations();
        }
        System.out.println(row.append(String.format("%7d", pins)));
      }
      System.out.printf("%-8s %-33s", kindLabel(kind), "  baseline cycle time");
      for (double b : baseCycle) {
        System.out.printf("%20s", String.format("%.2f s", b));
      }
      System.out.println();
    }
  }

  /** Auto focus: nearest-opponent vs threat-based selection with hysteresis. Whole-team effect. */
  static void runAutoFocusComparison(double seconds) throws IOException {
    System.out.printf("=== Auto focus (no opponent chosen): %.0f s, effect on ALL opponents ===%n",
        seconds);
    System.out.printf("%-10s %-34s %12s %8s %10s %6s%n",
        "Opponents", "Setup", "Total cyc", "Slowed", "Changes/min", "Pins");
    for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
      SimSession base = run(new Setup("No defense", -1, false, true), kind, seconds, null);
      int baseTotal = totalCycles(base);
      System.out.printf("%-10s %-34s %12d %8s %10s %6s%n", kindLabel(kind), "No defense", baseTotal,
          "-", "-", "0");
      for (Setup setup : autoFocusSetups()) {
        SimSession s = run(setup, kind, seconds, null);
        int total = totalCycles(s);
        System.out.printf("%-10s %-34s %12d %+7.0f%% %10.1f %6d%n", kindLabel(kind), setup.label(),
            total, 100.0 * (baseTotal - total) / total,
            s.metrics().targetSwitches() / (seconds / 60.0), s.world().pinViolations());
      }
    }
  }

  static List<Setup> autoFocusSetups() {
    List<Setup> list = new ArrayList<>();
    list.add(new Setup("Shadow + hold, locked on #0", 0, true, true, SimSession.Sensor.PERFECT, 1, 0, true));
    list.add(new Setup("Shadow + hold, auto=nearest", 0, true, true, SimSession.Sensor.PERFECT, 1, -1, false));
    list.add(new Setup("Shadow + hold, auto=threat", 0, true, true, SimSession.Sensor.PERFECT, 1, -1, true));
    list.add(new Setup("Line mirror + hold, locked on #0", 3, true, true, SimSession.Sensor.PERFECT, 1, 0, true));
    list.add(new Setup("Line mirror + hold, auto=nearest", 3, true, true, SimSession.Sensor.PERFECT, 1, -1, false));
    list.add(new Setup("Line mirror + hold, auto=threat", 3, true, true, SimSession.Sensor.PERFECT, 1, -1, true));
    return list;
  }

  static int totalCycles(SimSession s) {
    int total = 0;
    for (SimWorld.Opponent o : s.world().opponents()) {
      total += o.behavior().cyclesCompleted();
    }
    return total;
  }

  static String kindLabel(Scenarios.OpponentKind kind) {
    return kind == Scenarios.OpponentKind.EVADER ? "Evader" : "Cycler";
  }

  /** Seeds averaged for each noisy-sensor row. */
  static final int SEEDS = 5;

  /** How much worse do the two best setups get with a realistic or bad camera? */
  static void runTrackingComparison(double seconds) throws IOException {
    System.out.printf("=== Tracking quality: %.0f s, guarding #%d, noisy rows averaged over %d seeds ===%n",
        seconds, GUARDED_ID, SEEDS);
    System.out.printf("%-10s %-22s %-10s %8s %6s %10s %9s%n",
        "Opponents", "Setup", "Sensor", "Slowed", "Pins", "Est. err", "Coverage");
    for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
      double base = run(new Setup("none", -1, false, true), kind, seconds, null)
          .world().opponents().get(GUARDED_ID).behavior().averageCycleSeconds();
      int[] modes = {0, 3};
      String[] names = {"Shadow + hold", "Line mirror + hold"};
      for (int i = 0; i < modes.length; i++) {
        for (SimSession.Sensor sensor : SimSession.Sensor.values()) {
          int n = sensor == SimSession.Sensor.PERFECT ? 1 : SEEDS;
          double cycle = 0, err = 0, cov = 0;
          int pins = 0;
          for (long seed = 1; seed <= n; seed++) {
            SimSession s = run(new Setup(names[i], modes[i], true, true, sensor, seed), kind, seconds, null);
            cycle += s.world().opponents().get(GUARDED_ID).behavior().averageCycleSeconds();
            pins += s.world().pinViolations();
            if (sensor != SimSession.Sensor.PERFECT) {
              err += s.metrics().meanEstimateError();
              cov += s.metrics().trackCoverage();
            }
          }
          cycle /= n;
          String errText = sensor == SimSession.Sensor.PERFECT ? "0" : String.format("%.2f m", err / n);
          String covText = sensor == SimSession.Sensor.PERFECT ? "100%" : String.format("%.0f%%", 100 * cov / n);
          System.out.printf("%-10s %-22s %-10s %+7.0f%% %6s %10s %9s%n",
              kind == Scenarios.OpponentKind.EVADER ? "Evader" : "Cycler", names[i], sensor.label(),
              100 * (cycle - base) / base,
              n == 1 ? String.valueOf(pins) : String.format("%.1f", (double) pins / n),
              errText, covText);
        }
      }
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
      String cycleText = Double.isNaN(cycle) ? "jammed" : String.format("%.2f s", cycle);
      System.out.printf("%-34s %12s %8s %8d %7.1f s %6d %10d%n",
          setup.label(), cycleText, slowed, total, s.world().selfContactSeconds(),
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
