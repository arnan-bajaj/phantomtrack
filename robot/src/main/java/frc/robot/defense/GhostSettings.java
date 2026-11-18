package frc.robot.defense;

import phantom.sim.Scenarios;

/**
 * Which ghost opponents to run. Chosen on the dashboard.
 *
 * @param layout field layout (Open, Choke, Two lanes): the simulator's own made-up layouts
 * @param kind Cycler (drives through us) or Evader (dodges us)
 * @param count how many ghosts, 0 to 3 (one per opponent pickup spot)
 * @param maxSpeed ghost top speed (m/s)
 */
public record GhostSettings(
    Scenarios.Layout layout, Scenarios.OpponentKind kind, int count, double maxSpeed) {

  public static final int MAX_COUNT = 3;

  public GhostSettings {
    count = Math.max(0, Math.min(MAX_COUNT, count));
    maxSpeed = Double.isFinite(maxSpeed) ? Math.max(0.1, Math.min(4.5, maxSpeed)) : 1.0;
  }

  /** One slow Cycler on the open field: the gentlest thing to defend against. */
  public static GhostSettings firstTest() {
    return new GhostSettings(Scenarios.Layout.OPEN, Scenarios.OpponentKind.CYCLER, 1, 1.0);
  }
}
