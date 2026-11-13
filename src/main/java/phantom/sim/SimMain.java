package phantom.sim;

import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;
import phantom.core.control.ChassisCommand;
import phantom.sim.ui.FieldRenderer;
import phantom.sim.ui.SimWindow;

/**
 * Entry point.
 *
 * <pre>
 *   ./gradlew run                                   interactive window
 *   ./gradlew run --args="--headless"               60 s comparison, prints metrics, saves PNG
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

  /** Runs the same scenario with no defense and with Shadow defense, and compares. */
  static void runHeadless(double seconds) throws IOException {
    int steps = (int) Math.round(seconds / SimSession.DT);

    SimSession baseline = SimSession.standard();
    for (int i = 0; i < steps; i++) {
      baseline.step(ChassisCommand.STOP, false); // our robot parks
    }

    SimSession shadow = SimSession.standard();
    BufferedImage midShot = null;
    for (int i = 0; i < steps; i++) {
      shadow.step(ChassisCommand.STOP, true); // assist drives the whole time
      if (i == steps / 2) {
        midShot = render(shadow);
      }
    }

    System.out.printf("PhantomTrack headless run: %.0f s, mode = %s%n%n", seconds,
        shadow.assist().activeMode().name());
    System.out.printf("%-28s %12s %12s%n", "", "No defense", "Shadow");
    for (int i = 0; i < baseline.world().opponents().size(); i++) {
      OpponentBehavior b0 = baseline.world().opponents().get(i).behavior();
      OpponentBehavior b1 = shadow.world().opponents().get(i).behavior();
      System.out.printf("%-28s %12d %12d%n", "Opponent #" + i + " cycles",
          b0.cyclesCompleted(), b1.cyclesCompleted());
    }
    SimSession.Metrics m = shadow.metrics();
    System.out.printf("%-28s %12s %11.0f%%%n", "Goal-side of guarded opp", "-",
        100 * m.goalSideFraction());
    System.out.printf("%-28s %12s %10.2f m%n", "Mean tracking error", "-", m.meanTrackingError());
    System.out.printf("%-28s %12s %10.1f s%n", "Contact time", "-",
        shadow.world().selfContactSeconds());

    File dir = new File("snapshots");
    dir.mkdirs();
    File out = new File(dir, "shadow-midrun.png");
    ImageIO.write(midShot, "png", out);
    System.out.println();
    System.out.println("Saved " + out.getPath());
  }

  static BufferedImage render(SimSession s) {
    BufferedImage img = new BufferedImage(1320, 780, BufferedImage.TYPE_INT_RGB);
    Graphics2D g = img.createGraphics();
    new FieldRenderer().paint(g, img.getWidth(), img.getHeight(), s, false);
    g.dispose();
    return img;
  }
}
