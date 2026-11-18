package phantom.sim;

import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
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
 *   ./gradlew run --args="--anonymous 180 5"        only the anonymous-tracking tables (s, seeds)
 *   ./gradlew run --args="--batch-anon 180 5"       anonymous vs known-id tracking, partners on/off,
 *                                                   open / choke / two lanes -> results/batch-anon.csv
 *   ./gradlew run --args="--tracker-sweep 180 5"    MultiTargetTracker settings compared (how its
 *                                                   defaults were chosen) + camera coverage
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
    if (args.length > 0 && args[0].equals("--batch-anon")) {
      double seconds = args.length > 1 ? Double.parseDouble(args[1]) : 180.0;
      int seeds = args.length > 2 ? Integer.parseInt(args[2]) : SEEDS;
      BatchRunner.runAnonymous(seconds, seeds, new File("results", "batch-anon.csv"));
      return;
    }
    if (args.length > 0 && args[0].equals("--anonymous")) {
      runAnonymousComparison(args.length > 1 ? Double.parseDouble(args[1]) : 180.0,
          args.length > 2 ? Integer.parseInt(args[2]) : SEEDS);
      return;
    }
    if (args.length > 0 && args[0].equals("--tracker-sweep")) {
      double sweepSeconds = args.length > 1 ? Double.parseDouble(args[1]) : 180.0;
      int sweepSeeds = args.length > 2 ? Integer.parseInt(args[2]) : SEEDS;
      runHandoffSweep(sweepSeconds, sweepSeeds);
      System.out.println();
      runConfirmationSweep(sweepSeconds, sweepSeeds);
      System.out.println();
      runCameraCoverageSweep(sweepSeconds, sweepSeeds);
      System.out.println();
      runCoverageBreakdown(sweepSeconds, sweepSeeds);
      System.out.println();
      runPersistentFalsePositives(sweepSeconds, sweepSeeds);
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
      long seed, int focus, boolean threatSelector, Scenarios.Layout layout,
      SimSession.Tracking tracking, boolean partners, SimSession.Variant variant) {
    Setup(String label, int modeIndex, boolean hold, boolean guard, SimSession.Sensor sensor,
        long seed, int focus, boolean threatSelector, Scenarios.Layout layout) {
      this(label, modeIndex, hold, guard, sensor, seed, focus, threatSelector, layout,
          SimSession.Tracking.KNOWN_ID, false, SimSession.Variant.STANDARD);
    }

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
      return new Setup(label, modeIndex, hold, guard, s, newSeed, focus, threatSelector, layout,
          tracking, partners, variant);
    }

    Setup withLayout(Scenarios.Layout l) {
      return new Setup(label, modeIndex, hold, guard, sensor, seed, focus, threatSelector, l,
          tracking, partners, variant);
    }

    Setup withTracking(SimSession.Tracking t, boolean withPartners, SimSession.Variant v) {
      return new Setup(label, modeIndex, hold, guard, sensor, seed, focus, threatSelector, layout,
          t, withPartners, v);
    }
  }

  /** The standard list of setups compared in headless runs. */
  static List<Setup> standardSetups() {
    List<Setup> setups = new ArrayList<>();
    setups.add(new Setup("No defense", -1, false, true));
    String[] names = {"Shadow", "Interpose", "Zone", "Line mirror", "Cut-off"};
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
            ? new SimSession(Scenarios.three(
                field, kind, Scenarios.parkedOutOfTheWay(field), setup.partners()))
            : SimSession.standard(kind, setup.sensor(), setup.seed(), setup.layout(),
                setup.partners(), setup.tracking(), setup.variant());
    if (setup.modeIndex() >= 0) {
      s.assist().setModeIndex(setup.modeIndex());
      s.assist().setHoldEnabled(setup.hold());
      if (setup.focus() >= 0) {
        s.lockOn(setup.focus()); // known ids: focus = that id; anonymous: simulated driver points
      } else {
        s.assist().setFocusOpponentId(setup.focus());
      }
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
    System.out.println();
    runAnonymousComparison(seconds);
  }

  // ---------------- anonymous tracking ----------------

  /** The two setups of the anonymous-tracking comparison (focus: -1 = auto threat, 0 = #0). */
  static List<Setup> anonymousSetups() {
    return List.of(
        new Setup("Line mirror + hold, auto=threat", 3, true, true, SimSession.Sensor.PERFECT, 1,
            -1, true),
        new Setup("Cut-off + hold, locked #0", 4, true, true, SimSession.Sensor.PERFECT, 1,
            GUARDED_ID, true));
  }

  /** Layouts of the anonymous-tracking comparison. */
  static final Scenarios.Layout[] ANON_LAYOUTS = {Scenarios.Layout.OPEN, Scenarios.Layout.CHOKE};

  /** Averages of one comparison row over its seeds. NaN inputs (e.g. a jammed cycle) are skipped. */
  static final class RowStats {
    final double[] sum = new double[16];
    final double[] sumSq = new double[16];
    final int[] n = new int[16];
    int runs = 0;

    static final int SLOWED = 0;
    static final int ALL_SLOWED = 1;
    static final int PINS = 2;
    static final int RMSE = 3;
    static final int ID_SWITCHES_PER_MIN = 4;
    static final int FALSE_S_PER_MIN = 5;
    static final int COVERED = 6;
    static final int PARTNER_S_PER_MIN = 7;
    static final int REPOINTS_PER_MIN = 8;
    static final int HANDOFFS_PER_MIN = 9;
    static final int PARTNER_CONTACT_S = 10;
    static final int FRESH_FALSE_S_PER_MIN = 11;
    static final int FRESH_PARTNER_S_PER_MIN = 12;
    static final int AIM_NOTHING_S_PER_MIN = 13;
    static final int AIM_PARTNER_S_PER_MIN = 14;

    void add(int k, double v) {
      if (!Double.isNaN(v)) {
        sum[k] += v;
        sumSq[k] += v * v;
        n[k]++;
      }
    }

    double mean(int k) {
      return n[k] == 0 ? Double.NaN : sum[k] / n[k];
    }

    /**
     * Standard error of the mean: s / sqrt(n), s = sample standard deviation over runs
     * (s^2 = (sum v^2 - n mean^2) / (n - 1)). NaN with fewer than 2 runs.
     */
    double stderr(int k) {
      if (n[k] < 2) {
        return Double.NaN;
      }
      double m = mean(k);
      double var = Math.max(0.0, (sumSq[k] - n[k] * m * m) / (n[k] - 1));
      return Math.sqrt(var / n[k]);
    }

    void addRun(SimSession s, double seconds, double baseCycle, int baseTotal) {
      runs++;
      double minutes = seconds / 60.0;
      double cycle = s.world().opponents().get(GUARDED_ID).behavior().averageCycleSeconds();
      int total = totalCycles(s);
      add(SLOWED, 100 * (cycle - baseCycle) / baseCycle);
      add(ALL_SLOWED, 100.0 * (baseTotal - total) / total);
      add(PINS, s.world().pinViolations());
      SimSession.Metrics m = s.metrics();
      add(RMSE, m.positionRmse());
      add(ID_SWITCHES_PER_MIN, m.idSwitches() / minutes);
      add(FALSE_S_PER_MIN, m.falseTrackSeconds() / minutes);
      add(FRESH_FALSE_S_PER_MIN, m.freshFalseTrackSeconds() / minutes);
      add(FRESH_PARTNER_S_PER_MIN, m.freshPartnerAsOpponentSeconds() / minutes);
      add(AIM_NOTHING_S_PER_MIN, m.guardingNothingSeconds() / minutes);
      add(AIM_PARTNER_S_PER_MIN, m.guardingPartnerSeconds() / minutes);
      add(COVERED, m.meanCoverage());
      add(PARTNER_S_PER_MIN, m.partnerAsOpponentSeconds() / minutes);
      if (s.tracking() == SimSession.Tracking.ANONYMOUS) {
        add(REPOINTS_PER_MIN, s.redesignations() / minutes);
        add(HANDOFFS_PER_MIN,
            (s.multiTracker().handoffs() + s.multiTracker().eliminations()) / minutes);
      }
      add(PARTNER_CONTACT_S, s.world().selfPartnerContactSeconds());
    }
  }

  /**
   * Known-ID Kalman vs MultiTargetTracker with alliance partners on the field, plus the effect of
   * re-acquisition. Camera rows are averaged over {@code seeds} seeds.
   */
  static void runAnonymousComparison(double seconds) throws IOException {
    runAnonymousComparison(seconds, SEEDS);
  }

  static void runAnonymousComparison(double seconds, int seeds) throws IOException {
    Map<String, RowStats> rows = new LinkedHashMap<>();
    for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
      for (Scenarios.Layout layout : ANON_LAYOUTS) {
        SimSession base = run(new Setup("No defense", -1, false, true).withLayout(layout)
            .withTracking(SimSession.Tracking.KNOWN_ID, true, SimSession.Variant.STANDARD),
            kind, seconds, null);
        double baseCycle = base.world().opponents().get(GUARDED_ID).behavior().averageCycleSeconds();
        int baseTotal = totalCycles(base);
        for (Setup setup : anonymousSetups()) {
          for (SimSession.Sensor sensor : SimSession.Sensor.values()) {
            for (SimSession.Tracking tracking : SimSession.Tracking.values()) {
              for (SimSession.Variant variant : SimSession.Variant.values()) {
                if (!anonymousRowExists(sensor, tracking, variant)) {
                  continue;
                }
                RowStats stats = new RowStats();
                int n = sensor == SimSession.Sensor.PERFECT ? 1 : seeds;
                for (long seed = 1; seed <= n; seed++) {
                  Setup one = setup.withLayout(layout).withSensor(sensor, seed)
                      .withTracking(tracking, true, variant);
                  stats.addRun(run(one, kind, seconds, null), seconds, baseCycle, baseTotal);
                }
                rows.put(rowKey(kind, layout, setup, sensor, tracking, variant), stats);
              }
            }
          }
        }
      }
    }

    System.out.printf("=== Anonymous tracking, defense: %.0f s, 2 alliance partners on the field,"
        + " camera rows averaged over %d seeds ===%n", seconds, seeds);
    System.out.println("Slowed = guarded opponent #0 +- standard error over seeds (all opponents'"
        + " cycles). PtnrCt = s/run our robot touched a partner. Aim none/ptnr = s/min the assist"
        + " reacted to a track on no robot / on a partner. Repnt/m = times/min the driver"
        + " re-points the locked assist.");
    System.out.printf("%-7s %-6s %-31s %-9s %-9s %16s %7s %5s %7s %14s %7s%n", "Opps", "Field",
        "Setup", "Sensor", "Tracking", "Slowed", "(all)", "Pins", "PtnrCt", "Aim none/ptnr",
        "Repnt/m");
    forEachAnonymousRow(rows, (label, st) -> System.out.printf("%s %16s %7s %5s %7s %14s %7s%n",
        label, pct(st.mean(RowStats.SLOWED)) + " +-" + num(st.stderr(RowStats.SLOWED), 0),
        pct(st.mean(RowStats.ALL_SLOWED)), num(st.mean(RowStats.PINS), 1),
        num(st.mean(RowStats.PARTNER_CONTACT_S), 1),
        num(st.mean(RowStats.AIM_NOTHING_S_PER_MIN), 1) + " / "
            + num(st.mean(RowStats.AIM_PARTNER_S_PER_MIN), 1),
        num(st.mean(RowStats.REPOINTS_PER_MIN), 1)));
    System.out.println();

    System.out.printf("=== Anonymous tracking, tracking quality (same runs) ===%n");
    System.out.println("Scored without ids (CLEAR-MOT): a reported track within "
        + SimSession.Metrics.MATCH_RADIUS + " m of a robot is on it. False/m, Partn/m ="
        + " track-seconds per minute on no robot / on a partner (fresh tracks only).");
    System.out.printf("%-7s %-6s %-31s %-9s %-9s %5s %8s %11s %6s %10s%n", "Opps", "Field",
        "Setup", "Sensor", "Tracking", "RMSE", "IDsw/min", "False/m", "Cover", "Partn/m");
    forEachAnonymousRow(rows, (label, st) -> System.out.printf("%s %5s %8s %11s %6s %10s%n",
        label, num(st.mean(RowStats.RMSE), 2), num(st.mean(RowStats.ID_SWITCHES_PER_MIN), 1),
        num(st.mean(RowStats.FALSE_S_PER_MIN), 1) + " ("
            + num(st.mean(RowStats.FRESH_FALSE_S_PER_MIN), 1) + ")",
        pctPlain(st.mean(RowStats.COVERED)),
        num(st.mean(RowStats.PARTNER_S_PER_MIN), 1) + " ("
            + num(st.mean(RowStats.FRESH_PARTNER_S_PER_MIN), 1) + ")"));
    System.out.println();
    runReacquisitionTable(rows);
  }

  /** Perfect tracking is ground truth with ids (one reference row); variants are anonymous only. */
  static boolean anonymousRowExists(SimSession.Sensor sensor, SimSession.Tracking tracking,
      SimSession.Variant variant) {
    if (sensor == SimSession.Sensor.PERFECT) {
      return tracking == SimSession.Tracking.KNOWN_ID && variant == SimSession.Variant.STANDARD;
    }
    return tracking == SimSession.Tracking.ANONYMOUS || variant == SimSession.Variant.STANDARD;
  }

  /** Calls {@code print} with the row label and stats of every STANDARD-variant row, in order. */
  private static void forEachAnonymousRow(Map<String, RowStats> rows,
      java.util.function.BiConsumer<String, RowStats> print) {
    for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
      for (Scenarios.Layout layout : ANON_LAYOUTS) {
        for (Setup setup : anonymousSetups()) {
          for (SimSession.Sensor sensor : SimSession.Sensor.values()) {
            for (SimSession.Tracking tracking : SimSession.Tracking.values()) {
              RowStats st = rows.get(rowKey(kind, layout, setup, sensor, tracking,
                  SimSession.Variant.STANDARD));
              if (st == null) {
                continue;
              }
              String trackingLabel = sensor == SimSession.Sensor.PERFECT ? "-" : tracking.label();
              print.accept(String.format("%-7s %-6s %-31s %-9s %-9s", kindLabel(kind),
                  layout.field().name(), setup.label(), sensor.label(), trackingLabel), st);
            }
          }
        }
      }
    }
  }

  /** One re-acquisition setting tried by the sweep. */
  record HandoffOption(String label, java.util.function.UnaryOperator<
      phantom.core.tracking.MultiTargetTracker.Config> tuning) {}

  static List<HandoffOption> handoffOptions() {
    double inf = Double.POSITIVE_INFINITY;
    return List.of(
        new HandoffOption("off", c -> c.withoutHandoff()),
        new HandoffOption("r1.5 lead0.5 v3 (1st guess)",
            c -> c.withHandoff(2.0, 1.5, 3.0, 0.5).withElimination(0, 0.0)),
        new HandoffOption("r1.5 lead0.5 v-",
            c -> c.withHandoff(2.0, 1.5, inf, 0.5).withElimination(0, 0.0)),
        new HandoffOption("r1.5 lead1.0 v-",
            c -> c.withHandoff(2.0, 1.5, inf, 1.0).withElimination(0, 0.0)),
        new HandoffOption("r2.5 lead0.5 v-",
            c -> c.withHandoff(2.0, 2.5, inf, 0.5).withElimination(0, 0.0)),
        new HandoffOption("r3.5 lead1.0 v- w3",
            c -> c.withHandoff(3.0, 3.5, inf, 1.0).withElimination(0, 0.0)),
        new HandoffOption("r2.5 + elim 4 s (default)",
            c -> c.withHandoff(2.0, 2.5, inf, 0.5).withElimination(3, 4.0)),
        new HandoffOption("r2.5 + elim 8 s",
            c -> c.withHandoff(2.0, 2.5, inf, 0.5).withElimination(3, 8.0)));
  }

  /**
   * Tries re-acquisition settings on the anonymous comparison's runs (partners on, both opponent
   * kinds, open + choke, Realistic + Bad) and prints the totals for each setting, per setup.
   */
  static void runHandoffSweep(double seconds, int seeds) throws IOException {
    System.out.printf("=== Re-acquisition sweep: %.0f s, %d seeds x 2 opponent kinds x 2 fields x"
        + " Realistic+Bad, partners on ===%n", seconds, seeds);
    System.out.printf("%-31s %-27s %13s %9s %9s %9s %7s %6s%n", "Setup", "Re-acquisition",
        "Slowed", "IDsw/min", "Repnt/m", "Handof/m", "Pins", "Cover");
    for (Setup setup : anonymousSetups()) {
      for (HandoffOption option : handoffOptions()) {
        RowStats st = pooled(setup,
            new SimSession.Sensor[] {SimSession.Sensor.REALISTIC, SimSession.Sensor.BAD},
            seconds, seeds, (w, cam, seed) -> new SimSession(w, cam, seed,
                SimSession.Tracking.ANONYMOUS, option.tuning()));
        int slowed = setup.focus() >= 0 ? RowStats.SLOWED : RowStats.ALL_SLOWED;
        System.out.printf("%-31s %-27s %13s %9s %9s %9s %7s %6s%n", setup.label(), option.label(),
            pct(st.mean(slowed)) + " +-" + num(st.stderr(slowed), 1),
            num(st.mean(RowStats.ID_SWITCHES_PER_MIN), 1),
            setup.focus() >= 0 ? num(st.mean(RowStats.REPOINTS_PER_MIN), 1) : "-",
            num(st.mean(RowStats.HANDOFFS_PER_MIN), 1), num(st.mean(RowStats.PINS), 2),
            pctPlain(st.mean(RowStats.COVERED)));
      }
    }
  }

  /** Makes one session for a sweep: (world, camera, seed) -> session. */
  interface SessionMaker {
    SimSession make(SimWorld world, SimSession.Sensor camera, long seed);
  }

  /**
   * Runs {@code setup} over both opponent kinds, open + choke, the given cameras and seeds, with
   * partners on, and pools the results.
   */
  static RowStats pooled(Setup setup, SimSession.Sensor[] cameras, double seconds, int seeds,
      SessionMaker maker) throws IOException {
    RowStats st = new RowStats();
    for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
      for (Scenarios.Layout layout : ANON_LAYOUTS) {
        SimSession base = run(new Setup("No defense", -1, false, true).withLayout(layout)
            .withTracking(SimSession.Tracking.KNOWN_ID, true, SimSession.Variant.STANDARD),
            kind, seconds, null);
        double baseCycle = base.world().opponents().get(GUARDED_ID).behavior().averageCycleSeconds();
        int baseTotal = totalCycles(base);
        for (SimSession.Sensor camera : cameras) {
          for (long seed = 1; seed <= seeds; seed++) {
            SimWorld w = Scenarios.three(layout.field(), kind, Scenarios.DEFENDER_START, true);
            SimSession s = maker.make(w, camera, seed);
            s.assist().setModeIndex(setup.modeIndex());
            s.assist().setHoldEnabled(setup.hold());
            if (setup.focus() >= 0) {
              s.lockOn(setup.focus());
            } else {
              s.autoFocus();
            }
            int steps = (int) Math.round(seconds / SimSession.DT);
            for (int i = 0; i < steps; i++) {
              s.step(ChassisCommand.STOP, true);
            }
            st.addRun(s, seconds, baseCycle, baseTotal);
          }
        }
      }
    }
    return st;
  }

  /** M-of-N confirmation settings, per camera (they run at 30 Hz and 15 Hz). */
  static void runConfirmationSweep(double seconds, int seeds) throws IOException {
    System.out.printf("=== Confirmation sweep (M of N), anonymous tracking: %.0f s, %d seeds x 2"
        + " opponent kinds x 2 fields, partners on, both setups pooled ===%n", seconds, seeds);
    System.out.printf("%-9s %-8s %13s %9s %8s %11s %10s %9s%n", "Camera", "M of N", "Slowed(all)",
        "IDsw/min", "Cover", "FreshFalse", "FreshPtnr", "Repnt/m");
    int[][] options = {{3, 5}, {2, 4}, {2, 3}};
    for (SimSession.Sensor camera : new SimSession.Sensor[] {
        SimSession.Sensor.REALISTIC, SimSession.Sensor.BAD}) {
      for (int[] mn : options) {
        RowStats all = new RowStats();
        RowStats locked = null;
        for (Setup setup : anonymousSetups()) {
          RowStats st = pooled(setup, new SimSession.Sensor[] {camera}, seconds, seeds,
              (w, cam, seed) -> new SimSession(w, cam, seed, SimSession.Tracking.ANONYMOUS,
                  c -> c.withConfirmation(mn[0], mn[1])));
          if (setup.focus() >= 0) {
            locked = st;
          } else {
            all = st;
          }
        }
        System.out.printf("%-9s %-8s %13s %9s %8s %11s %10s %9s%n", camera.label(),
            mn[0] + " of " + mn[1] + (mn[0] == 3 && mn[1] == 5 ? "*" : ""),
            pct(all.mean(RowStats.ALL_SLOWED)) + " +-" + num(all.stderr(RowStats.ALL_SLOWED), 1),
            num(all.mean(RowStats.ID_SWITCHES_PER_MIN), 1), pctPlain(all.mean(RowStats.COVERED)),
            num(all.mean(RowStats.FRESH_FALSE_S_PER_MIN), 1),
            num(all.mean(RowStats.FRESH_PARTNER_S_PER_MIN), 1),
            num(locked.mean(RowStats.REPOINTS_PER_MIN), 1));
      }
    }
    System.out.println("(* = default. Slowed, IDsw/min, Cover, Fresh* = Line mirror + hold,"
        + " auto=threat; Repnt/m = Cut-off + hold, locked #0.)");
  }

  /** Does seeing more of the field fix identity? The Realistic camera with 140 vs 360 degrees. */
  static void runCameraCoverageSweep(double seconds, int seeds) throws IOException {
    System.out.printf("=== Camera coverage: Realistic camera with 140 vs 360 degrees of view,"
        + " %.0f s, %d seeds x 2 opponent kinds x 2 fields, partners on ===%n", seconds, seeds);
    System.out.printf("%-31s %-9s %-10s %13s %9s %8s %9s%n", "Setup", "View", "Tracking",
        "Slowed", "IDsw/min", "Cover", "Repnt/m");
    AnonymousSensor.Config narrow = AnonymousSensor.Config.realistic();
    NoisySensor.Config c = narrow.camera();
    AnonymousSensor.Config wide = new AnonymousSensor.Config(
        new NoisySensor.Config("Realistic 360", c.noiseStd(), c.latencySec(), c.dropout(), 360.0,
            c.rangeM(), c.rateHz()),
        narrow.colorErrorRate(), narrow.clutterPerFrame(), narrow.clutterOpponentShare(),
        narrow.selfDetectionRate(), narrow.occlusion());
    for (Setup setup : anonymousSetups()) {
      for (AnonymousSensor.Config cam : List.of(narrow, wide)) {
        for (SimSession.Tracking tracking : new SimSession.Tracking[] {
            SimSession.Tracking.KNOWN_ID, SimSession.Tracking.ANONYMOUS}) {
          RowStats st = pooled(setup, new SimSession.Sensor[] {SimSession.Sensor.REALISTIC},
              seconds, seeds, (w, label, seed) -> SimSession.withCamera(w, label, cam, seed,
                  tracking, x -> x));
          int slowed = setup.focus() >= 0 ? RowStats.SLOWED : RowStats.ALL_SLOWED;
          System.out.printf("%-31s %-9s %-10s %13s %9s %8s %9s%n", setup.label(),
              (int) cam.camera().fovDeg() + " deg", tracking.label(),
              pct(st.mean(slowed)) + " +-" + num(st.stderr(slowed), 1),
              num(st.mean(RowStats.ID_SWITCHES_PER_MIN), 1), pctPlain(st.mean(RowStats.COVERED)),
              num(st.mean(RowStats.REPOINTS_PER_MIN), 1));
        }
      }
    }
    System.out.println("(Known-ID here = the old camera model: no occlusion, ids handed over.)");
  }

  /**
   * Where coverage goes between the old known-id model and anonymous tracking, one change at a
   * time (Line mirror + hold, auto=threat).
   */
  static void runCoverageBreakdown(double seconds, int seeds) throws IOException {
    System.out.printf("=== Coverage breakdown, Line mirror + hold auto=threat: %.0f s, %d seeds x 2"
        + " opponent kinds x 2 fields, partners on ===%n", seconds, seeds);
    System.out.printf("%-9s %-58s %6s %5s %13s%n", "Camera", "Pipeline", "Cover", "RMSE",
        "Slowed(all)");
    Setup lm = anonymousSetups().get(0);
    for (SimSession.Sensor camera : new SimSession.Sensor[] {
        SimSession.Sensor.REALISTIC, SimSession.Sensor.BAD}) {
      AnonymousSensor.Config full = camera.anonymous;
      AnonymousSensor.Config clean =
          full.withClutter(0.0, 0.5).withSelfDetectionRate(0.0).withColorError(0.0);
      List<String> labels = List.of(
          "known ids, old camera (no occlusion)",
          "known ids + occlusion",
          "anonymous (the full new camera)",
          "anonymous, no false/self detections, no color errors",
          "anonymous, same, and confirmed on the first detection");
      List<SessionMaker> makers = List.of(
          (w, cam, seed) -> new SimSession(w, cam, seed, SimSession.Tracking.KNOWN_ID,
              SimSession.Variant.STANDARD),
          (w, cam, seed) -> new SimSession(w, cam, seed, SimSession.Tracking.KNOWN_ID_OCCLUSION,
              SimSession.Variant.STANDARD),
          (w, cam, seed) -> new SimSession(w, cam, seed, SimSession.Tracking.ANONYMOUS,
              SimSession.Variant.STANDARD),
          (w, cam, seed) -> SimSession.withCamera(w, cam, clean, seed,
              SimSession.Tracking.ANONYMOUS, c -> c),
          (w, cam, seed) -> SimSession.withCamera(w, cam, clean, seed,
              SimSession.Tracking.ANONYMOUS, c -> c.withConfirmation(1, 1)));
      for (int i = 0; i < labels.size(); i++) {
        RowStats st = pooled(lm, new SimSession.Sensor[] {camera}, seconds, seeds, makers.get(i));
        System.out.printf("%-9s %-58s %6s %5s %13s%n", camera.label(), labels.get(i),
            pctPlain(st.mean(RowStats.COVERED)), num(st.mean(RowStats.RMSE), 2),
            pct(st.mean(RowStats.ALL_SLOWED)) + " +-" + num(st.stderr(RowStats.ALL_SLOWED), 1));
      }
    }
  }

  /**
   * The realistic worst case for confirmation: false positives that repeat in the same place
   * (Realistic camera otherwise unchanged, anonymous tracking).
   */
  static void runPersistentFalsePositives(double seconds, int seeds) throws IOException {
    System.out.printf("=== Persistent false positives (fixed spots, each detected 30%% of frames"
        + " while in view), Realistic camera, anonymous: %.0f s, %d seeds x 2 opponent kinds x 2"
        + " fields, partners on ===%n", seconds, seeds);
    System.out.printf("%-31s %-6s %13s %9s %11s %12s %8s %9s%n", "Setup", "Spots", "Slowed",
        "IDsw/min", "FreshFalse", "AimNone/min", "Cover", "Repnt/m");
    for (Setup setup : anonymousSetups()) {
      for (int spots : new int[] {0, 3, 6}) {
        AnonymousSensor.Config cam =
            AnonymousSensor.Config.realistic().withPersistentFalsePositives(spots, 0.3);
        RowStats st = pooled(setup, new SimSession.Sensor[] {SimSession.Sensor.REALISTIC},
            seconds, seeds, (w, label, seed) -> SimSession.withCamera(w, label, cam, seed,
                SimSession.Tracking.ANONYMOUS, c -> c));
        int slowed = setup.focus() >= 0 ? RowStats.SLOWED : RowStats.ALL_SLOWED;
        System.out.printf("%-31s %-6d %13s %9s %11s %12s %8s %9s%n", setup.label(), spots,
            pct(st.mean(slowed)) + " +-" + num(st.stderr(slowed), 1),
            num(st.mean(RowStats.ID_SWITCHES_PER_MIN), 1),
            num(st.mean(RowStats.FRESH_FALSE_S_PER_MIN), 1),
            num(st.mean(RowStats.AIM_NOTHING_S_PER_MIN), 1), pctPlain(st.mean(RowStats.COVERED)),
            setup.focus() >= 0 ? num(st.mean(RowStats.REPOINTS_PER_MIN), 1) : "-");
      }
    }
  }

  static String rowKey(Scenarios.OpponentKind kind, Scenarios.Layout layout, Setup setup,
      SimSession.Sensor sensor, SimSession.Tracking tracking, SimSession.Variant variant) {
    return kind + "|" + layout + "|" + setup.label() + "|" + sensor + "|" + tracking + "|"
        + variant;
  }

  /** Same anonymous runs with re-acquisition (id handoff) on vs off. */
  static void runReacquisitionTable(Map<String, RowStats> rows) {
    System.out.println("=== Re-acquisition (id handoff by proximity + elimination), anonymous"
        + " tracking: on vs off (same seeds) ===");
    System.out.printf("%-7s %-6s %-31s %-9s %16s %12s %16s %14s %12s %10s%n", "Opps", "Field",
        "Setup", "Sensor", "Slowed on/off", "Pins on/off", "IDsw/min on/off", "Repnt/m on/off",
        "Cover on/off", "Handoff/m");
    for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
      for (Scenarios.Layout layout : ANON_LAYOUTS) {
        for (Setup setup : anonymousSetups()) {
          for (SimSession.Sensor sensor : new SimSession.Sensor[] {
              SimSession.Sensor.REALISTIC, SimSession.Sensor.BAD}) {
            RowStats on = rows.get(rowKey(kind, layout, setup, sensor,
                SimSession.Tracking.ANONYMOUS, SimSession.Variant.STANDARD));
            RowStats off = rows.get(rowKey(kind, layout, setup, sensor,
                SimSession.Tracking.ANONYMOUS, SimSession.Variant.NO_HANDOFF));
            boolean guarded = setup.focus() >= 0;
            int slowed = guarded ? RowStats.SLOWED : RowStats.ALL_SLOWED;
            System.out.printf("%-7s %-6s %-31s %-9s %16s %12s %16s %14s %12s %10s%n",
                kindLabel(kind), layout.field().name(), setup.label(), sensor.label(),
                pct(on.mean(slowed)) + " / " + pct(off.mean(slowed)),
                num(on.mean(RowStats.PINS), 1) + " / " + num(off.mean(RowStats.PINS), 1),
                num(on.mean(RowStats.ID_SWITCHES_PER_MIN), 1) + " / "
                    + num(off.mean(RowStats.ID_SWITCHES_PER_MIN), 1),
                guarded ? num(on.mean(RowStats.REPOINTS_PER_MIN), 1) + " / "
                    + num(off.mean(RowStats.REPOINTS_PER_MIN), 1) : "-",
                pctPlain(on.mean(RowStats.COVERED)) + " / " + pctPlain(off.mean(RowStats.COVERED)),
                num(on.mean(RowStats.HANDOFFS_PER_MIN), 1));
          }
        }
      }
    }
    System.out.println("(Slowed is the guarded opponent for 'locked #0' rows, all opponents for"
        + " 'auto' rows.)");
  }

  static String pct(double v) {
    return Double.isNaN(v) ? "jam" : String.format("%+.0f%%", v);
  }

  static String pctPlain(double fraction) {
    return Double.isNaN(fraction) ? "-" : String.format("%.0f%%", 100 * fraction);
  }

  static String num(double v, int decimals) {
    return Double.isNaN(v) ? "-" : String.format("%." + decimals + "f", v);
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
        new Setup("Line mirror + hold, auto=threat", 3, true, true, SimSession.Sensor.PERFECT, 1, -1, true),
        new Setup("Cut-off + hold, locked #0", 4, true, true));
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
