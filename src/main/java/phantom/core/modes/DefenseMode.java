package phantom.core.modes;

import java.util.Optional;

/**
 * A defense strategy. Must be a pure function of the context: no hardware, no static state, no
 * clock reads. That is what lets the same code run in tests, in sim and on the robot.
 */
public interface DefenseMode {
  String name();

  /** Where we want to be, or empty if this mode has nothing to do (e.g. no opponents seen). */
  Optional<DefenseTarget> compute(DefenseContext ctx);
}
