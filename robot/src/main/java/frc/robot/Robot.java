package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Standard command-based robot shell. All PhantomTrack wiring is in {@link RobotContainer}. */
public class Robot extends TimedRobot {
  public Robot() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
