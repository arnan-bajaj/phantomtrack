package phantom.sim;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * The contact-sensing study: pins and slowed % for the two recommended setups with a noisy camera,
 * on every layout, averaged over seeds. Writes one CSV row per run and prints a summary table.
 *
 * <pre>
 *   ./gradlew classes
 *   java -cp build/classes/java/main phantom.sim.ContactStudy 180 10 results/contact-study.csv
 *
 *   slowed_pct = 100 * (guarded_avg_cycle - baseline_avg_cycle) / baseline_avg_cycle
 *                (baseline = no defense, same layout and opponent kind)
 * </pre>
 */
public final class ContactStudy {
  private ContactStudy() {}

  static final String HEADER =
      "layout,opponents,setup,sensor,seed,seconds,guarded_avg_cycle_s,baseline_avg_cycle_s,"
          + "slowed_pct,pins,backoffs,contact_s";

  private static final int[] MODES = {3, 4};
  private static final String[] NAMES = {"Line mirror + hold", "Cut-off + hold"};
  private static final SimSession.Sensor[] SENSORS = {
    SimSession.Sensor.REALISTIC, SimSession.Sensor.BAD
  };

  public static void main(String[] args) throws IOException {
    double seconds = args.length > 0 ? Double.parseDouble(args[0]) : 180.0;
    int seeds = args.length > 1 ? Integer.parseInt(args[1]) : 10;
    File out = new File(args.length > 2 ? args[2] : "results/contact-study.csv");
    run(seconds, seeds, out);
  }

  public static void run(double seconds, int seeds, File out) throws IOException {
    File dir = out.getAbsoluteFile().getParentFile();
    if (dir != null) {
      dir.mkdirs();
    }
    System.out.printf(
        "=== Contact study: %.0f s per run, %d seeds, guarding #%d ===%n",
        seconds, seeds, SimMain.GUARDED_ID);
    System.out.printf(
        "%-10s %-7s %-19s %-10s %8s %9s %8s %9s %10s%n",
        "Layout", "Opps", "Setup", "Sensor", "Slowed", "Pins/run", "Max pins", "Runs>0", "Back-offs");
    int totalPins = 0;
    int totalRuns = 0;
    try (PrintWriter w = new PrintWriter(out, StandardCharsets.UTF_8)) {
      w.println(HEADER);
      for (Scenarios.Layout layout : Scenarios.Layout.values()) {
        for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
          SimSession base =
              SimMain.run(
                  new SimMain.Setup("none", -1, false, true).withLayout(layout), kind, seconds, null);
          double baseCycle = BatchRunner.guardedCycle(base);
          for (int m = 0; m < MODES.length; m++) {
            for (SimSession.Sensor sensor : SENSORS) {
              double slowedSum = 0;
              int slowedRuns = 0;
              int pins = 0;
              int maxPins = 0;
              int runsWithPins = 0;
              int backoffs = 0;
              for (long seed = 1; seed <= seeds; seed++) {
                SimMain.Setup setup =
                    new SimMain.Setup(NAMES[m], MODES[m], true, true, sensor, seed)
                        .withLayout(layout);
                SimSession s = SimMain.run(setup, kind, seconds, null);
                double cycle = BatchRunner.guardedCycle(s);
                double slowed = 100 * (cycle - baseCycle) / baseCycle;
                if (!Double.isNaN(slowed)) {
                  slowedSum += slowed;
                  slowedRuns++;
                }
                int p = s.world().pinViolations();
                pins += p;
                maxPins = Math.max(maxPins, p);
                runsWithPins += p > 0 ? 1 : 0;
                backoffs += s.assist().backoffsTriggered();
                w.println(
                    String.join(
                        ",",
                        layout.field().name(),
                        SimMain.kindLabel(kind),
                        NAMES[m],
                        sensor.label(),
                        String.valueOf(seed),
                        fmt(seconds),
                        fmt(cycle),
                        fmt(baseCycle),
                        fmt(slowed),
                        String.valueOf(p),
                        String.valueOf(s.assist().backoffsTriggered()),
                        fmt(s.world().selfContactSeconds())));
              }
              totalPins += pins;
              totalRuns += seeds;
              System.out.printf(
                  "%-10s %-7s %-19s %-10s %+7.1f%% %9.2f %8d %6d/%-2d %10.1f%n",
                  layout.field().name(),
                  SimMain.kindLabel(kind),
                  NAMES[m],
                  sensor.label(),
                  slowedRuns == 0 ? Double.NaN : slowedSum / slowedRuns,
                  (double) pins / seeds,
                  maxPins,
                  runsWithPins,
                  seeds,
                  (double) backoffs / seeds);
            }
          }
        }
      }
    }
    System.out.printf(
        "%nTotal: %d pins in %d runs (%.2f per run). CSV: %s%n",
        totalPins, totalRuns, (double) totalPins / totalRuns, out.getPath());
  }

  private static String fmt(double v) {
    return Double.isNaN(v) ? "" : String.format(Locale.ROOT, "%.3f", v);
  }
}
