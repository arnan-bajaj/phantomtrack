package frc.robot.defense;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.manager.DefenseAssist;
import phantom.sim.Scenarios;

/**
 * How long the robot stays in contact with ghosts, with a perfect drivetrain that applies each
 * command LATENCY later. Not a unit test: run it on purpose, with program arguments (no -D system
 * properties: Gradle's daemon keeps those between builds).
 *
 * <pre>
 *   ./gradlew pinMarginStudy --args="[seconds=300] [out=build/pin-margin-study] [only=text]"
 * </pre>
 *
 * {@code only} keeps the setups whose label contains that text (e.g. {@code only="3 CO"}).
 *
 * <p>Grid: speed cap 0.5 / 1.0 m/s (2 m/s^2) and 4.5 m/s (6 m/s^2, the sim robot) x command
 * latency 0 / 100 ms x Cyclers / Evaders x 1 / 3 ghosts (ghost top speed = our cap) x Line mirror
 * + hold / Cut-off + hold, first-test area (5 x 5 m, 0.75 m fence margin).
 *
 * <p>Two contact times per ghost and episode (an episode ends at 1.83 m apart):
 *
 * <ul>
 *   <li>"raw" = GhostSession's any-touch timer: bumper gap at most 0.05 m, whoever pushes (what
 *       the pin guard counts, so it includes being pushed);
 *   <li>"referee" = the same, but only while we drive into that ghost (effort above 0.1): what a
 *       referee counts. A referee pin is an episode whose referee time reaches 3 s.
 * </ul>
 *
 * <p>Each episode whose raw contact time passes 2.5 s is put in one class: LATE (already past
 * 2.5 s when the first back-off of the episode started), CHASED (most of the time after 2.5 s was
 * during a back-off with the ghost still touching), INTO_OTHER (during that back-off we touched
 * another ghost), RECONTACT (most of it came from a new touch after a back-off ended), OTHER.
 *
 * <p>Effect on the ghosts: the same setup is run with our robot parked in the far corner
 * ({@code Scenarios.parkedOutOfTheWay}, as the simulator's "No defense"), for the baseline.
 *
 * <pre>
 *   slowed %    = 100 * (baseline cycles - cycles) / cycles   (all ghosts; = rise in mean cycle time)
 *   gridlocked  = some ghost finished no cycle, or we touched ghosts for more than half the run
 * </pre>
 */
public final class PinMarginStudy {
  static final double DT = 0.02;
  static final double LIMIT = 2.5;

  record Config(double cap, double accel, int latencyLoops, Scenarios.OpponentKind kind, int count,
      int mode) {
    String label() {
      return String.format(Locale.ROOT, "%.1f m/s %3d ms %-6s %d %s", cap, latencyLoops * 20,
          kind == Scenarios.OpponentKind.CYCLER ? "Cycler" : "Evader", count,
          mode == GhostSession.MODE_LINE_MIRROR ? "LM+hold" : "CO+hold");
    }
  }

  /** One ghost's open episode. */
  static final class Episode {
    double peak;
    double atFirstBackoff = -1;
    boolean backedOff;
    boolean touchedOtherDuringBackoff;
    double overLimitBackingOffTouching;
    double overLimitAfterBackoff;
    double overLimitOther;
    double overLimitYielding;
    double gainBackingMoving, gainYielding, gainEngagedAfter;
    int touchingOthersMax;
  }

  record Result(Config config, double peak, double refPeak, double maxAfterTrigger, int over25,
      int over30, int late, int chased, int intoOther, int recontact, int other, int backoffs,
      int refPins, int cycles, int minGhostCycles, double contactSeconds, List<String> worstTimeline,
      double worstPeak, Breakdown breakdown) {}

  /**
   * Where the contact time of the episodes over 2.5 s came from (s), summed over them.
   *
   * @param crowded episodes that touched another ghost while over 2.5 s
   * @param over contact time past 2.5 s
   * @param overYielding of that, while the guard was yielding
   * @param beforeBackoff contact before the first back-off of the episode
   * @param backingOffMoving contact after that while backing off on an exit
   * @param backingOffYielding contact after that while yielding
   * @param engagedAgain contact after that with the mode back in charge
   */
  record Breakdown(int crowded, double over, double overYielding, double beforeBackoff,
      double backingOffMoving, double backingOffYielding, double engagedAgain) {
    Breakdown plus(Breakdown o) {
      return new Breakdown(crowded + o.crowded, over + o.over, overYielding + o.overYielding,
          beforeBackoff + o.beforeBackoff, backingOffMoving + o.backingOffMoving,
          backingOffYielding + o.backingOffYielding, engagedAgain + o.engagedAgain);
    }
  }

  /** Cycles of each ghost with our robot parked in a corner, by (cap, kind, count). */
  private static final Map<String, Integer> BASELINE = new HashMap<>();

  static int baselineCycles(Config c, double seconds) {
    String key = c.cap() + "/" + c.accel() + "/" + c.kind() + "/" + c.count() + "/" + seconds;
    return BASELINE.computeIfAbsent(key, k -> {
      PhantomTrack pt = new PhantomTrack(safety(c), new GhostSettings(Scenarios.Layout.OPEN,
          c.kind(), c.count(), c.cap()));
      Pose parked = Scenarios.parkedOutOfTheWay(Scenarios.Layout.OPEN.field());
      int steps = (int) Math.round(seconds / DT);
      for (int i = 0; i < steps; i++) {
        pt.update(new PhantomTrack.Input(DT * i, RobotMode.TELEOP, parked, Vec2.ZERO, DT * i,
            ChassisCommand.STOP, 0.0, 0.0));
      }
      int total = 0;
      for (var o : pt.session().world().opponents()) {
        total += o.behavior().cyclesCompleted();
      }
      return total;
    });
  }

  static SafetyConfig safety(Config c) {
    SafetyConfig base = SafetyConfig.firstTest();
    return base.withTunables(c.cap(), c.accel(), base.areaMinX(), base.areaMinY(),
        base.areaMaxX(), base.areaMaxY(), base.fenceMargin());
  }

  static Result run(Config c, double seconds, boolean keepTimeline) {
    PhantomTrack pt = new PhantomTrack(safety(c), new GhostSettings(Scenarios.Layout.OPEN, c.kind(),
        c.count(), c.cap()));
    Pose pose = new Pose(10.0, 4.035, Math.PI);
    int k = 0;
    for (int i = 0; i < 40; i++, k++) {
      pt.update(new PhantomTrack.Input(DT * k, RobotMode.TELEOP, pose, Vec2.ZERO, DT * k,
          ChassisCommand.STOP, 0.0, 0.0));
    }
    pt.selectMode(c.mode());
    pt.toggleHold();
    for (int i = 0; i < 5; i++, k++) {
      pt.update(new PhantomTrack.Input(DT * k, RobotMode.TELEOP, pose, Vec2.ZERO, DT * k,
          ChassisCommand.STOP, 0.0, 0.0));
    }
    ArrayDeque<ChassisCommand> pipe = new ArrayDeque<>();
    for (int i = 0; i < c.latencyLoops(); i++) {
      pipe.add(ChassisCommand.STOP);
    }
    Vec2 v = Vec2.ZERO;
    int n = c.count();
    Episode[] ep = new Episode[n];
    List<Episode> done = new ArrayList<>();
    double[] prev = new double[n];
    boolean wasBacking = false;
    ArrayDeque<String> hist = new ArrayDeque<>();
    List<String> worst = List.of();
    double worstPeak = 0.0;
    double peak = 0.0;
    double refPeak = 0.0;
    double contact = 0.0;
    int steps = (int) Math.round(seconds / DT);
    for (int i = 0; i < steps; i++, k++) {
      ChassisCommand out = pt.update(new PhantomTrack.Input(DT * k, RobotMode.TELEOP, pose, v,
          DT * k, ChassisCommand.STOP, 0.0, 1.0));
      pipe.add(out);
      ChassisCommand applied = pipe.poll();
      v = applied.velocity();
      pose = new Pose(pose.position().plus(v.times(DT)),
          Angles.wrap(pose.heading() + applied.omega() * DT));
      GhostSession s = pt.session();
      double[] ps = pt.pinSeconds();
      boolean backing = pt.coreState() == DefenseAssist.State.BACKING_OFF;
      boolean[] touching = new boolean[n];
      boolean anyTouch = false;
      for (int id = 0; id < n; id++) {
        double gap = s.world().opponents().get(id).body().position().distanceTo(pose.position())
            - 2.0 * s.field().robotHalfSize();
        touching[id] = gap <= 0.05;
        anyTouch |= touching[id];
        refPeak = Math.max(refPeak, s.world().refereeContactSeconds(id));
      }
      if (anyTouch) {
        contact += DT;
      }
      for (int id = 0; id < n; id++) {
        if (ps[id] > 0.0 && ep[id] == null) {
          ep[id] = new Episode();
        }
        Episode e = ep[id];
        if (e == null) {
          continue;
        }
        if (backing && !wasBacking && !e.backedOff) {
          e.atFirstBackoff = ps[id];
        }
        if (backing) {
          e.backedOff = true;
          for (int j = 0; j < n; j++) {
            e.touchedOtherDuringBackoff |= j != id && touching[j] && ps[id] > LIMIT;
          }
        }
        double added = ps[id] - prev[id];
        if (added > 0.0 && e.backedOff) {
          if (!backing) {
            e.gainEngagedAfter += added;
          } else if (s.assist().guardYielding()) {
            e.gainYielding += added;
          } else {
            e.gainBackingMoving += added;
          }
        }
        if (added > 0.0 && ps[id] > LIMIT) {
          double over = Math.min(added, ps[id] - LIMIT);
          if (s.assist().guardYielding()) {
            e.overLimitYielding += over;
          }
          int others = 0;
          for (int j = 0; j < n; j++) {
            others += j != id && touching[j] ? 1 : 0;
          }
          e.touchingOthersMax = Math.max(e.touchingOthersMax, others);
          if (backing && touching[id]) {
            e.overLimitBackingOffTouching += over;
          } else if (e.backedOff && !backing) {
            e.overLimitAfterBackoff += over;
          } else {
            e.overLimitOther += over;
          }
        }
        e.peak = Math.max(e.peak, ps[id]);
        peak = Math.max(peak, ps[id]);
        if (ps[id] == 0.0) {
          done.add(e);
          ep[id] = null;
        }
        prev[id] = ps[id];
      }
      wasBacking = backing;
      if (keepTimeline) {
        StringBuilder line = new StringBuilder(String.format(Locale.ROOT,
            "t=%7.2f %-12s self=(%5.2f,%5.2f) |v|=%.2f", DT * i, pt.coreState(),
            pose.position().x(), pose.position().y(), v.norm()));
        for (int id = 0; id < n; id++) {
          var b = s.world().opponents().get(id).body();
          line.append(String.format(Locale.ROOT,
              " | g%d (%5.2f,%5.2f) gap=%5.2f t=%4.2f ref=%4.2f guard=%4.2f", id, b.position().x(),
              b.position().y(), b.position().distanceTo(pose.position())
                  - 2.0 * s.field().robotHalfSize(), ps[id], s.world().refereeContactSeconds(id),
              s.assist().pinSeconds(id)));
        }
        hist.addLast(line.toString());
        if (hist.size() > 600) {
          hist.removeFirst();
        }
        for (int id = 0; id < n; id++) {
          if (ps[id] > worstPeak) {
            worstPeak = ps[id];
            worst = new ArrayList<>(hist);
          }
        }
      }
    }
    for (Episode e : ep) {
      if (e != null) {
        done.add(e);
      }
    }
    int over25 = 0, over30 = 0, late = 0, chased = 0, intoOther = 0, recontact = 0, other = 0;
    double maxAfterTrigger = 0.0;
    Breakdown breakdown = new Breakdown(0, 0, 0, 0, 0, 0, 0);
    for (Episode e : done) {
      if (e.backedOff && e.atFirstBackoff >= 0) {
        maxAfterTrigger = Math.max(maxAfterTrigger, e.peak - e.atFirstBackoff);
      }
      if (e.peak <= LIMIT + 1e-6) {
        continue;
      }
      over25++;
      breakdown = breakdown.plus(new Breakdown(e.touchingOthersMax > 0 ? 1 : 0,
          e.overLimitBackingOffTouching + e.overLimitAfterBackoff + e.overLimitOther,
          e.overLimitYielding, Math.max(0, e.atFirstBackoff), e.gainBackingMoving, e.gainYielding,
          e.gainEngagedAfter));
      if (e.peak > 3.0 + 1e-6) {
        over30++;
      }
      if (e.atFirstBackoff > LIMIT || (!e.backedOff)) {
        late++;
      } else if (e.touchedOtherDuringBackoff) {
        intoOther++;
      } else if (e.overLimitBackingOffTouching >= e.overLimitAfterBackoff
          && e.overLimitBackingOffTouching >= e.overLimitOther) {
        chased++;
      } else if (e.overLimitAfterBackoff >= e.overLimitOther) {
        recontact++;
      } else {
        other++;
      }
    }
    int cycles = 0;
    int minGhost = Integer.MAX_VALUE;
    for (var o : pt.session().world().opponents()) {
      cycles += o.behavior().cyclesCompleted();
      minGhost = Math.min(minGhost, o.behavior().cyclesCompleted());
    }
    return new Result(c, peak, refPeak, maxAfterTrigger, over25, over30, late, chased, intoOther,
        recontact, other, pt.session().assist().backoffsTriggered(), pt.pinViolations(), cycles,
        minGhost, contact, worst, worstPeak, breakdown);
  }

  static boolean gridlocked(Result r, double seconds) {
    return r.minGhostCycles() == 0 || r.contactSeconds() > seconds / 2.0;
  }

  static double slowedPct(Result r, double seconds) {
    int base = baselineCycles(r.config(), seconds);
    return r.cycles() == 0 ? Double.NaN : 100.0 * (base - r.cycles()) / r.cycles();
  }

  static List<Config> grid() {
    List<Config> out = new ArrayList<>();
    double[][] speeds = {{0.5, 2.0}, {1.0, 2.0}, {4.5, 6.0}};
    for (double[] sp : speeds) {
      for (int lat : new int[] {0, 5}) {
        for (Scenarios.OpponentKind kind : Scenarios.OpponentKind.values()) {
          for (int count : new int[] {1, 3}) {
            for (int mode : new int[] {GhostSession.MODE_LINE_MIRROR, GhostSession.MODE_CUTOFF}) {
              out.add(new Config(sp[0], sp[1], lat, kind, count, mode));
            }
          }
        }
      }
    }
    return out;
  }

  public static void main(String[] args) throws IOException {
    double seconds = 300.0;
    Path dir = Path.of("build/pin-margin-study");
    String only = "";
    for (String a : args) {
      if (a.startsWith("seconds=")) {
        seconds = Double.parseDouble(a.substring(8));
      } else if (a.startsWith("out=")) {
        dir = Path.of(a.substring(4));
      } else if (a.startsWith("only=")) {
        only = a.substring(5).replace('_', ' '); // "_" stands for a space
      } else {
        throw new IllegalArgumentException("unknown argument " + a
            + " (use seconds=, out=, only=)");
      }
    }
    Files.createDirectories(dir);
    StringBuilder table = new StringBuilder();
    table.append(String.format(Locale.ROOT,
        "%-34s %6s %6s %6s %5s %5s | %4s %6s %5s %5s %5s | %8s %6s %7s %5s %7s%n",
        "cap   latency opps   n setup", "peak", "refPk", "after", ">2.5", ">3.0", "late", "chased",
        "into", "recon", "other", "backoffs", "refPin", "slowed", "grid", "touch_s"));
    Result worst = null;
    int total25 = 0;
    List<Result> results = new ArrayList<>();
    for (Config c : grid()) {
      if (!c.label().contains(only)) {
        continue;
      }
      Result r = run(c, seconds, true);
      results.add(r);
      total25 += r.over25();
      table.append(String.format(Locale.ROOT,
          "%-34s %6.2f %6.2f %6.2f %5d %5d | %4d %6d %5d %5d %5d | %8d %6d %+6.0f%% %5s %7.1f%n",
          c.label(), r.peak(), r.refPeak(), r.maxAfterTrigger(), r.over25(), r.over30(), r.late(),
          r.chased(), r.intoOther(), r.recontact(), r.other(), r.backoffs(), r.refPins(),
          slowedPct(r, seconds), gridlocked(r, seconds) ? "YES" : "-", r.contactSeconds()));
      if (worst == null || r.worstPeak() > worst.worstPeak()) {
        worst = r;
      }
    }
    Breakdown b = new Breakdown(0, 0, 0, 0, 0, 0, 0);
    for (Result r : results) {
      b = b.plus(r.breakdown());
    }
    table.append("episodes over 2.5 s in total: ").append(total25).append('\n');
    table.append(String.format(Locale.ROOT,
        "  touching another ghost while over 2.5 s: %d; contact past 2.5 s: %.1f s (yielding %.1f s)%n"
            + "  their contact: before the first back-off %.1f s; after it, backing off on an exit"
            + " %.1f s, yielding %.1f s, engaged again %.1f s%n%n", b.crowded(), b.over(),
        b.overYielding(), b.beforeBackoff(), b.backingOffMoving(), b.backingOffYielding(),
        b.engagedAgain()));
    table.append(summary(results, seconds));
    System.out.print(table);
    Files.writeString(dir.resolve("summary.txt"), table.toString());
    if (worst != null) {
      Files.writeString(dir.resolve("worst-timeline.txt"),
          worst.config().label() + ", peak " + worst.worstPeak() + "\n"
              + String.join("\n", worst.worstTimeline()) + "\n");
    }
  }

  /** One row per (cap, ghost count), over latency x kind x mode (8 runs each). */
  static String summary(List<Result> results, double seconds) {
    StringBuilder sb = new StringBuilder();
    sb.append(String.format(Locale.ROOT, "%-8s %6s | %9s %9s %6s %9s %7s %10s%n", "cap",
        "ghosts", "raw peak", "ref peak", ">2.5", "ref pins", "slowed", "gridlocked"));
    for (double cap : new double[] {0.5, 1.0, 4.5}) {
      for (int count : new int[] {1, 3}) {
        double peak = 0, refPeak = 0, slowed = 0;
        int over = 0, pins = 0, grid = 0, runs = 0, slowedN = 0;
        for (Result r : results) {
          if (r.config().cap() != cap || r.config().count() != count) {
            continue;
          }
          runs++;
          peak = Math.max(peak, r.peak());
          refPeak = Math.max(refPeak, r.refPeak());
          over += r.over25();
          pins += r.refPins();
          grid += gridlocked(r, seconds) ? 1 : 0;
          double s = slowedPct(r, seconds);
          if (!Double.isNaN(s)) {
            slowed += s;
            slowedN++;
          }
        }
        if (runs == 0) {
          continue;
        }
        sb.append(String.format(Locale.ROOT, "%-8s %6d | %9.2f %9.2f %6d %9d %+6.0f%% %10s%n",
            cap + " m/s", count, peak, refPeak, over, pins,
            slowedN == 0 ? Double.NaN : slowed / slowedN, grid + " of " + runs));
      }
    }
    return sb.toString();
  }
}
