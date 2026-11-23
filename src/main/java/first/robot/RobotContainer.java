// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Command;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Command.noRequirements(
            coroutine -> {
              System.out.println("No autonomous routine configured");
            })
        .named("RobotContainer: Get Autonomous Command");
  }
}
