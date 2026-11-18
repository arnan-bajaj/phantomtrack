package phantom;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Test helper: finds references to forbidden packages in Java source files. Looks at code only
 * (comments are skipped), and catches both {@code import x.y.Z;} and fully qualified names used
 * inline such as {@code java.awt.Color c}.
 */
public final class SourceScan {
  private SourceScan() {}

  /** Every .java file under dir, sorted. */
  public static List<Path> javaFiles(Path dir) throws IOException {
    try (Stream<Path> s = Files.walk(dir)) {
      return s.filter(p -> p.toString().endsWith(".java")).sorted().toList();
    }
  }

  /**
   * @param packages package prefixes that must not appear, e.g. "java.awt"
   * @return one "file:line: text" entry per offending line (empty = clean)
   */
  public static List<String> violations(Path file, List<String> packages) throws IOException {
    return violations(file.toString(), Files.readString(file), packages);
  }

  /** Same check on source text, so the detector itself can be tested. */
  public static List<String> violations(String label, String source, List<String> packages) {
    StringBuilder alt = new StringBuilder();
    for (String p : packages) {
      alt.append(alt.length() == 0 ? "" : "|").append(Pattern.quote(p));
    }
    // Not preceded by a letter, digit, '_' or '.', so "myjava.awt" or "x.phantom.sim" don't match;
    // followed by a word boundary, so "phantom.simulation" doesn't match "phantom.sim".
    Pattern forbidden = Pattern.compile("(?<![\\w.])(" + alt + ")\\b");
    List<String> out = new ArrayList<>();
    String[] lines = source.split("\n", -1);
    List<String> code = codeLines(source);
    for (int i = 0; i < lines.length; i++) {
      Matcher m = forbidden.matcher(code.get(i));
      if (m.find()) {
        out.add(label + ":" + (i + 1) + ": " + lines[i].trim());
      }
    }
    return out;
  }

  /**
   * Simple class names (capitalized identifiers) used in the code of {@code source}, comments
   * and string literals left out. Used to check which classes of a package a file depends on
   * (same-package references need no import, so an import scan alone misses them).
   */
  public static java.util.Set<String> referencedNames(String source) {
    java.util.Set<String> out = new java.util.TreeSet<>();
    Pattern ident = Pattern.compile("\\b[A-Z][A-Za-z0-9_]*\\b");
    for (String line : codeLines(source)) {
      Matcher m = ident.matcher(line.replaceAll("\"(\\\\.|[^\"\\\\])*\"", "\"\""));
      while (m.find()) {
        out.add(m.group());
      }
    }
    return out;
  }

  /** The source split into lines with every comment removed (line numbers are kept). */
  public static List<String> codeLines(String source) {
    List<String> out = new ArrayList<>();
    boolean inBlock = false;
    String[] lines = source.split("\n", -1);
    for (int i = 0; i < lines.length; i++) {
      StringBuilder code = new StringBuilder();
      String line = lines[i];
      int pos = 0;
      while (pos < line.length()) {
        if (inBlock) {
          int end = line.indexOf("*/", pos);
          if (end < 0) {
            pos = line.length();
          } else {
            inBlock = false;
            pos = end + 2;
          }
        } else {
          int block = line.indexOf("/*", pos);
          int lineComment = line.indexOf("//", pos);
          if (lineComment >= 0 && (block < 0 || lineComment < block)) {
            code.append(line, pos, lineComment);
            pos = line.length();
          } else if (block >= 0) {
            code.append(line, pos, block);
            inBlock = true;
            pos = block + 2;
          } else {
            code.append(line, pos, line.length());
            pos = line.length();
          }
        }
      }
      out.add(code.toString());
    }
    return out;
  }
}
