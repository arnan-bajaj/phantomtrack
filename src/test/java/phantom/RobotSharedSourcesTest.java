package phantom;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The robot project (robot/) compiles some files straight from this project; the list is
 * robot/shared-sources.txt. Besides phantom/core (checked by CorePurityTest) it lists the
 * simulator's opponent classes, reused as "ghost" opponents on the real robot. Those run on a
 * roboRIO, so they must not touch AWT, Swing, image IO, WPILib, CTRE or the sim's UI package.
 */
class RobotSharedSourcesTest {
  static final Path LIST = Path.of("robot", "shared-sources.txt");
  static final Path SRC = Path.of("src", "main", "java");
  static final List<String> FORBIDDEN =
      List.of("java.awt", "javax.swing", "javax.imageio", "edu.wpi", "com.ctre", "phantom.sim.ui");

  private static List<String> patterns() throws IOException {
    List<String> out = new ArrayList<>();
    for (String line : Files.readAllLines(LIST)) {
      String s = line.trim();
      if (!s.isEmpty() && !s.startsWith("#")) {
        out.add(s);
      }
    }
    return out;
  }

  @Test
  void listSharesAllOfCoreAndOnlyNamedSimFiles() throws IOException {
    List<String> patterns = patterns();
    assertTrue(patterns.contains("phantom/core/**"), "core must be shared whole: " + patterns);
    for (String p : patterns) {
      if (p.equals("phantom/core/**")) {
        continue;
      }
      // Everything else must be one named sim file: no wildcards that could sweep in the UI.
      assertTrue(p.startsWith("phantom/sim/") && p.endsWith(".java"), "unexpected entry: " + p);
      assertFalse(p.contains("*"), "no wildcards outside core: " + p);
      assertFalse(p.startsWith("phantom/sim/ui/"), "the sim UI must never reach the robot: " + p);
      assertTrue(Files.isRegularFile(SRC.resolve(p)), "listed file does not exist: " + p);
    }
    assertTrue(patterns.size() > 1, "expected the ghost opponent files to be listed");
  }

  @Test
  void sharedSimFilesAreRobotSafe() throws IOException {
    List<String> bad = new ArrayList<>();
    for (String p : patterns()) {
      if (!p.equals("phantom/core/**")) {
        bad.addAll(SourceScan.violations(SRC.resolve(p), FORBIDDEN));
      }
    }
    assertTrue(
        bad.isEmpty(),
        () -> "files shared with robot/ must not depend on " + FORBIDDEN + ":\n  "
            + String.join("\n  ", bad));
  }

  @Test
  void theSimEntryPointWouldBeRejected() throws IOException {
    // Sanity check of the check: SimMain uses AWT, so listing it by mistake must be caught.
    List<String> bad = SourceScan.violations(SRC.resolve("phantom/sim/SimMain.java"), FORBIDDEN);
    assertFalse(bad.isEmpty());
  }
}
