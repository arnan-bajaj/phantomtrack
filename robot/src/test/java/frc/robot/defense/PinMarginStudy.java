package frc.robot.defense;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Angles;
import phantom.core.geometry.Pose;
import phantom.core.geometry.Vec2;
import phantom.core.manager.DefenseAssist;
import phantom.sim.Scenarios;

/**
 * How long the robot stays in contact with ghosts, with a perfect drivetrain that applies each
 * command LATENCY later. Not a unit test: run it on purpose.
 *
 * <pre>
 *   ./gradlew test --tests '*PinMarginStudy*' -Dpt.study=true [-Dpt.studyOut=dir] [-Dpt.studySeconds=300]
 * </pre>
 *
 * Grid: speed cap 0.5 / 1.0 m/s (2 m/s^2) and 4.5 m/s (6 m/s^2, the sim robot) x command latency 0
 * / 100 ms x Cyclers / Evaders x 1 / 3 ghosts (ghost top speed = our cap) x Line mirror + hold /
 * Cut-off + hold, first-test area (5 x 5 m, 0.75 m fence margin).
 *
 * <p>"Contact time" is GhostSession's any-touch timer: per ghost, time with a bumper gap of at
 * most 0.05 m, summed over an episode that only ends at 1.83 m apart (what the pin guard counts,
 * so it includes being pushed). The referee count (contact while we press in) is reported too.
 *
 * <p>Each episode whose contact time passes 2.5 s is put in one class:
 * "after" is the most contact time an episode gained after its first back-off started (how long a
 * back-off takes to break contact, plus any re-contact).
 *
 * <ul>
 *   <li>LATE: it had already passed 2.5 s when the first back-off of the episode started;
 *   <li>CHASED: most of the time after 2.5 s was during a back-off with the ghost still touching
 *       (it follows us);
 *   <li>INTO_OTHER: during that back-off we touched another ghost;
 *   <li>RECONTACT: most of the time after 2.5 s came from a new touch after a back-off ended;
 *   <li>OTHER.
 * </ul>
 */
@EnabledIfSystemProperty(named = "pt.study", matches = "true")
class PinMarginStudy {
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
  }

  record Result(Config config, double peak, double maxAfterTrigger, int over25, int over30, int late, int chased,
      int intoOther, int recontact, int other, int backoffs, int refPins, double cyclesPerMin,
      List<String> worstTimeline, double worstPeak) {}

  static Result run(Config c, double seconds, boolean keepTimeline) {
    SafetyConfig base = SafetyConfig.firstTest();
    SafetyConfig sc = base.withTunables(c.cap(), c.accel(), base.areaMinX(), base.areaMinY(),
        base.areaMaxX(), base.areaMaxY(), base.fenceMargin());
    PhantomTrack pt = new PhantomTrack(sc, new GhostSettings(Scenarios.Layout.OPEN, c.kind(),
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
      for (int id = 0; id < n; id++) {
        double gap = s.world().opponents().get(id).body().position().distanceTo(pose.position())
            - 2.0 * s.field().robotHalfSize();
        touching[id] = gap <= 0.05;
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
        if (added > 0.0 && ps[id] > LIMIT) {
          double over = Math.min(added, ps[id] - LIMIT);
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
          line.append(String.format(Locale.ROOT, " | g%d (%5.2f,%5.2f) gap=%5.2f t=%4.2f guard=%4.2f",
              id, b.position().x(), b.position().y(),
              b.position().distanceTo(pose.position()) - 2.0 * s.field().robotHalfSize(), ps[id],
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
    for (Episode e : done) {
      if (e.backedOff && e.atFirstBackoff >= 0) {
        maxAfterTrigger = Math.max(maxAfterTrigger, e.peak - e.atFirstBackoff);
      }
      if (e.peak <= LIMIT + 1e-6) {
        continue;
      }
      over25++;
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
    double cycles = 0;
    for (var o : pt.session().world().opponents()) {
      cycles += o.behavior().cyclesCompleted();
    }
    return new Result(c, peak, maxAfterTrigger, over25, over30, late, chased, intoOther, recontact, other,
        pt.session().assist().backoffsTriggered(), pt.pinViolations(), cycles / (seconds / 60.0),
        worst, worstPeak);
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

  @Test
  void study() throws IOException {
    double seconds = Double.parseDouble(System.getProperty("pt.studySeconds", "300"));
    Path dir = Path.of(System.getProperty("pt.studyOut", "build/pin-margin-study"));
    Files.createDirectories(dir);
    StringBuilder table = new StringBuilder();
    table.append(String.format(Locale.ROOT, "%-34s %6s %6s %5s %5s | %4s %6s %5s %5s %5s | %8s %6s %8s%n",
        "cap   latency opps   n setup", "peak", "after", ">2.5", ">3.0", "late", "chased", "into", "recon",
        "other", "backoffs", "refPin", "cyc/min"));
    Result worst = null;
    int total25 = 0;
    for (Config c : grid()) {
      Result r = run(c, seconds, true);
      total25 += r.over25();
      table.append(String.format(Locale.ROOT, "%-34s %6.2f %6.2f %5d %5d | %4d %6d %5d %5d %5d | %8d %6d %8.1f%n",
          c.label(), r.peak(), r.maxAfterTrigger(), r.over25(), r.over30(), r.late(), r.chased(), r.intoOther(),
          r.recontact(), r.other(), r.backoffs(), r.refPins(), r.cyclesPerMin()));
      if (worst == null || r.worstPeak() > worst.worstPeak()) {
        worst = r;
      }
    }
    table.append("episodes over 2.5 s in total: ").append(total25).append('\n');
    System.out.print(table);
    Files.writeString(dir.resolve("summary.txt"), table.toString());
    Files.writeString(dir.resolve("worst-timeline.txt"),
        worst.config().label() + ", peak " + worst.worstPeak() + "\n"
            + String.join("\n", worst.worstTimeline()) + "\n");
  }
}
