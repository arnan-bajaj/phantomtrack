package phantom.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import phantom.SourceScan;

/**
 * Architecture rules 1 and 2 in CLAUDE.md, enforced: phantom.core has zero dependencies, so the
 * robot project (robot/) can compile it unchanged. Fails if any file under phantom/core refers to
 * AWT, Swing, WPILib, CTRE or the simulator.
 */
class CorePurityTest {
  static final List<String> FORBIDDEN =
      List.of("java.awt", "javax.swing", "edu.wpi", "com.ctre", "phantom.sim");
  static final Path CORE = Path.of("src", "main", "java", "phantom", "core");

  @Test
  void coreImportsNothingFromAwtSwingWpilibCtreOrSim() throws IOException {
    assertTrue(Files.isDirectory(CORE), "run from the project root; missing " + CORE.toAbsolutePath());
    List<Path> files = SourceScan.javaFiles(CORE);
    // Guards against a vacuous pass (wrong folder, empty walk). Core had 36 files when this was
    // written; Vec2 is the one everything else depends on.
    assertTrue(files.size() >= 30, "expected the whole core tree, found only " + files.size() + " files");
    assertTrue(files.contains(CORE.resolve(Path.of("geometry", "Vec2.java"))));

    List<String> bad = new ArrayList<>();
    for (Path f : files) {
      bad.addAll(SourceScan.violations(f, FORBIDDEN));
    }
    assertTrue(
        bad.isEmpty(),
        () -> "phantom.core must not depend on " + FORBIDDEN + ":\n  " + String.join("\n  ", bad));
  }

  @Test
  void detectorCatchesImportsStaticImportsAndInlineQualifiedNames() {
    String src =
        String.join(
            "\n",
            "package phantom.core.x;", // line 1: clean
            "import java.awt.Color;", // line 2
            "import static javax.swing.SwingUtilities.invokeLater;", // line 3
            "import edu.wpi.first.math.geometry.Pose2d;", // line 4
            "import com.ctre.phoenix6.hardware.Pigeon2;", // line 5
            "import phantom.sim.SimWorld;", // line 6
            "class X { java.awt.Point p; }"); // line 7: no import, still caught
    List<String> bad = SourceScan.violations("X.java", src, FORBIDDEN);
    assertEquals(6, bad.size(), bad.toString());
    assertTrue(bad.get(0).startsWith("X.java:2:"));
    assertTrue(bad.get(5).startsWith("X.java:7:"));
  }

  @Test
  void detectorIgnoresCommentsAndLookalikeNames() {
    String src =
        String.join(
            "\n",
            "package phantom.core.x;",
            "/** In sim this is phantom.sim.RobotBody; on the robot it is edu.wpi stuff. */",
            "/* java.awt",
            "   javax.swing */",
            "import phantom.core.geometry.Vec2; // not java.awt",
            "import phantom.simulation.Thing;", // a different package, not phantom.sim
            "class X { int myjava; }");
    assertEquals(List.of(), SourceScan.violations("X.java", src, FORBIDDEN));
  }

  @Test
  void detectorSeesCodeAfterABlockCommentCloses() {
    String src = "/* note */ import java.awt.Color;\n/* a\n b */ import javax.swing.JFrame;";
    List<String> bad = SourceScan.violations("X.java", src, FORBIDDEN);
    assertEquals(2, bad.size(), bad.toString());
    assertTrue(bad.get(0).startsWith("X.java:1:"));
    assertTrue(bad.get(1).startsWith("X.java:3:"));
  }
}
