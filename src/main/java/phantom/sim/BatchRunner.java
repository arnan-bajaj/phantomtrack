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
 * in a spreadsheet. Perfect-sensor runs are deterministic, so they run once. Then the
 * anonymous-tracking grid ({@link SimMain#runAnonymousComparison}): alliance partners on, open
 * field and choke, known-id vs anonymous tracking, re-acquisition on and off.
 *
 * <pre>
 *   slowed_pct     = 100 * (guarded_avg_cycle - baseline_avg_cycle) / baseline_avg_cycle
 *   all_slowed_pct = 100 * (baseline_total_cycles - total_cycles) / total_cycles
 *   (the baseline is "No defense" on the same field, with partners if the row has them)
 * </pre>
 *
 * Tracking columns (pos_rmse_m onward) are scored without ids, the same way for every tracker:
 * see {@link SimSession#matchTracks}. est_error_m and track_coverage are the older id-based
 * numbers and are empty for anonymous rows (their track ids are not opponent ids).
 *
 * <p>contact_s is the total time our robot touched any opponent; peak_ref_contact_s is the
 * longest referee contact time in one pin episode (only while we drive into it);
 * hold_switches_per_min counts hold ground's push starting or stopping.
 */
public final class BatchRunner {
  private BatchRunner() {}

  static final String HEADER =
      "opponents,setup,mode,hold,pin_guard,sensor,focus,seed,seconds,"
          + "guarded_avg_cycle_s,baseline_avg_cycle_s,slowed_pct,total_cycles,"
          + "baseline_total_cycles,all_slowed_pct,contact_s,pins,backoffs,"
          + "est_error_m,track_coverage,target_switches,"
          + "layout,tracking,partners,variant,pos_rmse_m,id_switches_per_min,"
          + "false_track_s_per_min,fresh_false_track_s_per_min,coverage_opp0,coverage_opp1,"
          + "coverage_opp2,coverage_mean,partner_as_opp_s_per_min,"
          + "fresh_partner_as_opp_s_per_min,partner_contact_s,redesignations_per_min,handoffs,"
          + "eliminations,aim_nothing_s_per_min,aim_partner_s_per_min,peak_ref_contact_s,"
          + "hold_switches_per_min";

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
      rows += writeAnonymousGrid(w, seconds, seeds, summary, List.of(SimMain.ANON_LAYOUTS),
          List.of(SimSession.Tracking.values()), List.of(SimSession.Variant.values()), true,
          SimMain.anonymousSetups());
    }
    printSummary(rows, seconds, seeds, out, summary);
  }

  /**
   * Only the anonymous-tracking grid, on every field layout (open, choke, two lanes), with our two
   * alliance partners on the field and without them, for the two anonymous setups plus Line
   * mirror + hold locked on #0:
   * perfect tracking once, then known-id and anonymous tracking for each noisy camera and seed.
   * This is the grid behind README "Which mode when" (contact sensing is always on).
   */
  public static void runAnonymous(double seconds, int seeds, File out) throws IOException {
    File dir = out.getAbsoluteFile().getParentFile();
    if (dir != null) {
      dir.mkdirs();
    }
    Map<String, double[]> summary = new LinkedHashMap<>();
    // The two anonymous setups, plus Line mirror + hold locked on #0 so man-to-man can be compared
    // on the guarded opponent (README "Which mode when").
    List<SimMain.Setup> setups = new ArrayList<>(SimMain.anonymousSetups());
    setups.add(new SimMain.Setup("Line mirror + hold, locked #0", 3, true, true,
        SimSession.Sensor.PERFECT, 1, SimMain.GUARDED_ID, true));
    int rows;
    try (PrintWriter w = new PrintWriter(out, StandardCharsets.UTF_8)) {
      w.println(HEADER);
      rows = 0;
      for (boolean partners : new boolean[] {true, false}) {
        rows += writeAnonymousGrid(w, seconds, seeds, summary, List.of(Scenarios.Layout.values()),
            List.of(SimSession.Tracking.KNOWN_ID, SimSession.Tracking.ANONYMOUS),
            List.of(SimSession.Variant.STANDARD), partners, setups);
      }
    }
    printSummary(rows, seconds, seeds, out, summary);
  }

  private static void printSummary(
      int rows, double seconds, int seeds, File out, Map<String, double[]> summary) {
    System.out.printf("Wrote %d runs (%.0f s each, %d seeds for noisy sensors) to %s%n%n",
        rows, seconds, seeds, out.getPath());
    System.out.printf("%-58s %10s %10s%n", "Opponents | Setup | Sensor", "Slowed", "Pins/run");
    for (Map.Entry<String, double[]> e : summary.entrySet()) {
      double[] a = e.getValue();
      System.out.printf("%-58s %+9.0f%% %10.1f%n", e.getKey(), a[0] / a[2], a[1] / a[2]);
    }
  }

  /** The anonymous-tracking grid, partners on. Returns the number of rows written. */
  private static int writeAnonymousGrid(PrintWriter w, double seconds, int seeds,
      Map<String, double[]> summary, List<Scenarios.Layout> layouts,
      List<SimSession.Tracking> trackings, List<SimSession.Variant> variants, boolean partners,
      List<SimMain.Setup> setups) throws IOException {
    int rows = 0;
    for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
      for (Scenarios.Layout layout : layouts) {
        SimMain.Setup none = new SimMain.Setup("No defense", -1, false, true).withLayout(layout)
            .withTracking(SimSession.Tracking.KNOWN_ID, partners, SimSession.Variant.STANDARD);
        SimSession base = SimMain.run(none, kind, seconds, null);
        double baseCycle = guardedCycle(base);
        int baseTotal = SimMain.totalCycles(base);
        w.println(row(kind, none, base, seconds, baseCycle, baseTotal));
        rows++;
        for (SimMain.Setup setup : setups) {
          for (SimSession.Sensor sensor : SimSession.Sensor.values()) {
            for (SimSession.Tracking tracking : trackings) {
              for (SimSession.Variant variant : variants) {
                if (!SimMain.anonymousRowExists(sensor, tracking, variant)) {
                  continue;
                }
                int n = sensor == SimSession.Sensor.PERFECT ? 1 : seeds;
                for (long seed = 1; seed <= n; seed++) {
                  SimMain.Setup one = setup.withLayout(layout).withSensor(sensor, seed)
                      .withTracking(tracking, partners, variant);
                  SimSession s = SimMain.run(one, kind, seconds, null);
                  w.println(row(kind, one, s, seconds, baseCycle, baseTotal));
                  rows++;
                  String key = SimMain.kindLabel(kind) + " | " + layout.field().name()
                      + (partners ? " + partners | " : " | ") + setup.label() + " | " + sensor.label() + " | "
                      + tracking.label() + (variant == SimSession.Variant.STANDARD ? ""
                          : " | " + variant.name().toLowerCase());
                  double[] acc = summary.computeIfAbsent(key, k -> new double[3]);
                  double slowedNow = setup.focus() >= 0
                      ? 100 * (guardedCycle(s) - baseCycle) / baseCycle
                      : 100.0 * (baseTotal - SimMain.totalCycles(s)) / SimMain.totalCycles(s);
                  acc[0] += slowedNow;
                  acc[1] += s.world().pinViolations();
                  acc[2] += 1;
                }
              }
            }
          }
        }
      }
    }
    return rows;
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
        String.valueOf(s.metrics().targetSwitches()),
        quote(setup.layout().field().name()),
        noisy ? s.tracking().label() : "Perfect",
        String.valueOf(setup.partners()),
        setup.variant().name().toLowerCase(),
        fmt(s.metrics().positionRmse()),
        fmt(perMinute(s.metrics().idSwitches(), seconds)),
        fmt(perMinute(s.metrics().falseTrackSeconds(), seconds)),
        fmt(perMinute(s.metrics().freshFalseTrackSeconds(), seconds)),
        fmt(s.metrics().coverage(0)),
        fmt(s.metrics().coverage(1)),
        fmt(s.metrics().coverage(2)),
        fmt(s.metrics().meanCoverage()),
        fmt(perMinute(s.metrics().partnerAsOpponentSeconds(), seconds)),
        fmt(perMinute(s.metrics().freshPartnerAsOpponentSeconds(), seconds)),
        fmt(s.world().selfPartnerContactSeconds()),
        s.multiTracker() == null ? "" : fmt(perMinute(s.redesignations(), seconds)),
        s.multiTracker() == null ? "" : String.valueOf(s.multiTracker().handoffs()),
        s.multiTracker() == null ? "" : String.valueOf(s.multiTracker().eliminations()),
        fmt(perMinute(s.metrics().guardingNothingSeconds(), seconds)),
        fmt(perMinute(s.metrics().guardingPartnerSeconds(), seconds)),
        fmt(s.world().peakRefereeContactSeconds()),
        fmt(perMinute(s.assist().holdSwitches(), seconds)));
  }

  private static double perMinute(double count, double seconds) {
    return count / (seconds / 60.0);
  }

  private static String fmt(double v) {
    return Double.isNaN(v) ? "" : String.format(Locale.ROOT, "%.3f", v);
  }

  private static String quote(String s) {
    return s.contains(",") ? "\"" + s.replace("\"", "\"\"") + "\"" : s;
  }
}
