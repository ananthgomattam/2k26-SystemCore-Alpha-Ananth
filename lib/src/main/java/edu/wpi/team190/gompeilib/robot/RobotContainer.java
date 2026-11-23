package edu.wpi.team190.gompeilib.robot;

import org.wpilib.command3.Command;

public interface RobotContainer {

  public default void robotPeriodic() {}

  public default Command getAutonomousCommand() {
    return Command.noRequirements(corountine -> {}).named("Empty Autonomous Command");
  }
}
