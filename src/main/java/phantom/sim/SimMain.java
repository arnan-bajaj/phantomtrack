package phantom.sim;

import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import javax.imageio.ImageIO;
import phantom.core.control.ChassisCommand;
import phantom.sim.ui.FieldRenderer;
import phantom.sim.ui.SimWindow;

/**
 * Entry point.
 *
 * <pre>
 *   ./gradlew run                                   interactive window
 *   ./gradlew run --args="--headless"               60 s comparison of every mode, prints metrics, saves PNGs
 *   ./gradlew run --args="--headless 120"           same, 120 s
 * </pre>
 */
public final class SimMain {
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
    double seconds = args.length > 1 ? Double.parseDouble(args[1]) : 60.0;
    runHeadless(seconds);
  }

  /** Runs the same scenario with no defense and with each mode, and compares. */
  static void runHeadless(double seconds) throws IOException {
    int steps = (int) Math.round(seconds / SimSession.DT);
    File dir = new File("snapshots");
    dir.mkdirs();

    List<String> names = new ArrayList<>();
    List<SimSession> runs = new ArrayList<>();

    SimSession baseline = SimSession.standard();
    for (int i = 0; i < steps; i++) {
      baseline.step(ChassisCommand.STOP, false); // our robot parks
    }
    names.add("No defense");
    runs.add(baseline);

    int modeCount = baseline.assist().modes().size();
    for (int m = 0; m < modeCount; m++) {
      SimSession s = SimSession.standard();
      s.assist().setModeIndex(m);
      for (int i = 0; i < steps; i++) {
        s.step(ChassisCommand.STOP, true); // assist drives the whole time
        if (i == steps / 2) {
          String file = s.assist().activeMode().name().toLowerCase().replace(' ', '-') + "-midrun.png";
          ImageIO.write(render(s), "png", new File(dir, file));
        }
      }
      names.add(s.assist().activeMode().name());
      runs.add(s);
    }

    System.out.printf("PhantomTrack headless run: %.0f s, 3 Cycler opponents%n%n", seconds);
    StringBuilder header = new StringBuilder(String.format("%-26s", ""));
    for (String n : names) {
      header.append(String.format("%13s", n));
    }
    System.out.println(header);

    int oppCount = baseline.world().opponents().size();
    for (int o = 0; o < oppCount; o++) {
      StringBuilder row = new StringBuilder(String.format("%-26s", "Opponent #" + o + " cycles"));
      for (SimSession s : runs) {
        row.append(String.format("%13d", s.world().opponents().get(o).behavior().cyclesCompleted()));
      }
      System.out.println(row);
    }
    printRow("Total cycles", runs, true, s -> {
      int total = 0;
      for (SimWorld.Opponent o : s.world().opponents()) {
        total += o.behavior().cyclesCompleted();
      }
      return String.valueOf(total);
    });
    printRow("Goal-side when guarding", runs, false, s -> pct(s.metrics().goalSideFraction()));
    printRow("Mean tracking error", runs, false, s -> meters(s.metrics().meanTrackingError()));
    printRow("Contact time", runs, true, s -> String.format("%.1f s", s.world().selfContactSeconds()));

    System.out.println();
    System.out.println("Saved mid-run snapshots to " + dir.getPath() + "/");
  }

  /**
   * @param appliesToBaseline false for metrics that only exist while a mode is running
   */
  private static void printRow(
      String label, List<SimSession> runs, boolean appliesToBaseline, Function<SimSession, String> f) {
    StringBuilder row = new StringBuilder(String.format("%-26s", label));
    for (int i = 0; i < runs.size(); i++) {
      boolean skip = i == 0 && !appliesToBaseline;
      row.append(String.format("%13s", skip ? "-" : f.apply(runs.get(i))));
    }
    System.out.println(row);
  }

  private static String pct(double fraction) {
    return Double.isNaN(fraction) ? "-" : String.format("%.0f%%", 100 * fraction);
  }

  private static String meters(double m) {
    return Double.isNaN(m) ? "-" : String.format("%.2f m", m);
  }

  static BufferedImage render(SimSession s) {
    BufferedImage img = new BufferedImage(1320, 780, BufferedImage.TYPE_INT_RGB);
    Graphics2D g = img.createGraphics();
    new FieldRenderer().paint(g, img.getWidth(), img.getHeight(), s, false);
    g.dispose();
    return img;
  }
}
