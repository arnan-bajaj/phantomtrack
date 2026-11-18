package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Standard command-based robot shell. All PhantomTrack wiring is in {@link RobotContainer}.
 * DataLogManager records every NetworkTables value (all of /PhantomTrack) plus Driver Station and
 * joystick data to a .wpilog (roboRIO: /home/lvuser/logs or a USB stick; sim: robot/logs).
 */
public class Robot extends TimedRobot {
  private final RobotContainer container;

  public Robot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    container = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
