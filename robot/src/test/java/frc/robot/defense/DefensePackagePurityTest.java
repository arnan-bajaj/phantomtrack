package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;
import phantom.SourceScan;

/**
 * frc.robot.defense holds the safety rules and must stay testable without a robot and portable to
 * WPILib 2027: no WPILib, no vendor libraries, no UI, and no reaching up into the WPILib layer
 * (frc.robot.*). Tests run with the robot/ folder as the working directory.
 */
class DefensePackagePurityTest {
  static final Path DIR = Path.of("src", "main", "java", "frc", "robot", "defense");
  static final List<String> FORBIDDEN =
      List.of("edu.wpi", "com.ctre", "swervelib", "java.awt", "javax.swing", "phantom.sim.ui");
  static final Pattern UP_IMPORT = Pattern.compile("^\\s*import\\s+(static\\s+)?frc\\.robot\\.(?!defense\\.)");

  @Test
  void defensePackageUsesNoWpilibAndDoesNotImportTheRobotLayer() throws IOException {
    assertTrue(Files.isDirectory(DIR), "missing " + DIR.toAbsolutePath());
    List<Path> files = SourceScan.javaFiles(DIR);
    assertTrue(files.contains(DIR.resolve("SafetyGate.java")), "wrong folder? " + files);
    List<String> bad = new ArrayList<>();
    for (Path f : files) {
      bad.addAll(SourceScan.violations(f, FORBIDDEN));
      List<String> lines = Files.readAllLines(f);
      for (int i = 0; i < lines.size(); i++) {
        if (UP_IMPORT.matcher(lines.get(i)).find()) {
          bad.add(f + ":" + (i + 1) + ": " + lines.get(i).trim());
        }
      }
    }
    assertTrue(bad.isEmpty(), () -> "frc.robot.defense must stay WPILib-free:\n  " + String.join("\n  ", bad));
  }
}
