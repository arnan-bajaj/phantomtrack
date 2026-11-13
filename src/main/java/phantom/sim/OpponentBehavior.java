package phantom.sim;

import phantom.core.control.ChassisCommand;

/** How a simulated opponent decides where to drive. */
public interface OpponentBehavior {
  String name();

  ChassisCommand decide(double timeSec, double dt, RobotBody me, SimWorld world);

  /** Number of completed scoring cycles so far. */
  int cyclesCompleted();

  /** Average seconds per completed cycle, or NaN if none completed yet. */
  double averageCycleSeconds();
}
