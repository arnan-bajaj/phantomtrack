package phantom.sim;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Runs every setup x opponent kind x sensor x seed and writes one CSV row per run, for graphing
 * in a spreadsheet. Perfect-sensor runs are deterministic, so they run once.
 *
 * <pre>
 *   slowed_pct     = 100 * (guarded_avg_cycle - baseline_avg_cycle) / baseline_avg_cycle
 *   all_slowed_pct = 100 * (baseline_total_cycles - total_cycles) / total_cycles
 * </pre>
 */
public final class BatchRunner {
  private BatchRunner() {}

  static final String HEADER =
      "opponents,setup,mode,hold,pin_guard,sensor,focus,seed,seconds,"
          + "guarded_avg_cycle_s,baseline_avg_cycle_s,slowed_pct,total_cycles,"
          + "baseline_total_cycles,all_slowed_pct,contact_s,pins,backoffs,"
          + "est_error_m,track_coverage,target_switches";

  public static void run(double seconds, int seeds, File out) throws IOException {
    File dir = out.getAbsoluteFile().getParentFile();
    if (dir != null) {
      dir.mkdirs();
    }
    List<SimMain.Setup> setups = new ArrayList<>(SimMain.standardSetups());
    setups.addAll(SimMain.autoFocusSetups());

    int rows = 0;
    // mean slowed % per (opponents, setup, sensor) for the console summary
    Map<String, double[]> summary = new LinkedHashMap<>();
    try (PrintWriter w = new PrintWriter(out, StandardCharsets.UTF_8)) {
      w.println(HEADER);
      for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
        SimSession base = SimMain.run(new SimMain.Setup("No defense", -1, false, true), kind, seconds, null);
        double baseCycle = guardedCycle(base);
        int baseTotal = SimMain.totalCycles(base);
        w.println(row(kind, SimMain.standardSetups().get(0), base, seconds, baseCycle, baseTotal));
        rows++;
        for (SimMain.Setup setup : setups) {
          if (setup.modeIndex() < 0) {
            continue;
          }
          for (SimSession.Sensor sensor : SimSession.Sensor.values()) {
            int n = sensor == SimSession.Sensor.PERFECT ? 1 : seeds;
            for (long seed = 1; seed <= n; seed++) {
              SimSession s = SimMain.run(setup.withSensor(sensor, seed), kind, seconds, null);
              w.println(row(kind, setup.withSensor(sensor, seed), s, seconds, baseCycle, baseTotal));
              rows++;
              String key = SimMain.kindLabel(kind) + " | " + setup.label() + " | " + sensor.label();
              double[] acc = summary.computeIfAbsent(key, k -> new double[3]);
              acc[0] += 100 * (guardedCycle(s) - baseCycle) / baseCycle;
              acc[1] += s.world().pinViolations();
              acc[2] += 1;
            }
          }
        }
      }
    }
    System.out.printf("Wrote %d runs (%.0f s each, %d seeds for noisy sensors) to %s%n%n",
        rows, seconds, seeds, out.getPath());
    System.out.printf("%-58s %10s %10s%n", "Opponents | Setup | Sensor", "Slowed", "Pins/run");
    for (Map.Entry<String, double[]> e : summary.entrySet()) {
      double[] a = e.getValue();
      System.out.printf("%-58s %+9.0f%% %10.1f%n", e.getKey(), a[0] / a[2], a[1] / a[2]);
    }
  }

  static double guardedCycle(SimSession s) {
    return s.world().opponents().get(SimMain.GUARDED_ID).behavior().averageCycleSeconds();
  }

  static String row(Scenarios.OpponentKind kind, SimMain.Setup setup, SimSession s, double seconds,
      double baseCycle, int baseTotal) {
    double cycle = guardedCycle(s);
    int total = SimMain.totalCycles(s);
    boolean defending = setup.modeIndex() >= 0;
    boolean noisy = s.sensor() != SimSession.Sensor.PERFECT;
    return String.join(",",
        SimMain.kindLabel(kind),
        quote(setup.label()),
        defending ? s.assist().modes().get(setup.modeIndex()).name() : "none",
        String.valueOf(setup.hold()),
        String.valueOf(setup.guard()),
        s.sensor().label(),
        !defending ? "none" : setup.focus() >= 0 ? "#" + setup.focus()
            : setup.threatSelector() ? "auto-threat" : "auto-nearest",
        String.valueOf(setup.seed()),
        fmt(seconds),
        fmt(cycle),
        fmt(baseCycle),
        defending ? fmt(100 * (cycle - baseCycle) / baseCycle) : "",
        String.valueOf(total),
        String.valueOf(baseTotal),
        defending ? fmt(100.0 * (baseTotal - total) / total) : "",
        fmt(s.world().selfContactSeconds()),
        String.valueOf(s.world().pinViolations()),
        String.valueOf(s.assist().backoffsTriggered()),
        noisy ? fmt(s.metrics().meanEstimateError()) : "0",
        noisy ? fmt(s.metrics().trackCoverage()) : "1",
        String.valueOf(s.metrics().targetSwitches()));
  }

  private static String fmt(double v) {
    return Double.isNaN(v) ? "" : String.format(Locale.ROOT, "%.3f", v);
  }

  private static String quote(String s) {
    return s.contains(",") ? "\"" + s.replace("\"", "\"\"") + "\"" : s;
  }
}
