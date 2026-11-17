package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Entry point. Do not add anything here. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
