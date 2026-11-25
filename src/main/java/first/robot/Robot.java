// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixUtil;
import edu.wpi.team190.gompeilib.robot.RobotContainer;
import edu.wpi.team190.gompeilib.robot.subsystems.VirtualSubsystem;
import first.robot.util.Elastic;
import first.robot.util.HubActivePeriod;
import first.robot.v3_Horse.V3_HorseRobotContainer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    super(Constants.LOOP_PERIOD_SECONDS);

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        // setUseTiming(false);
        Logger.addDataReceiver(new NT4Publisher());
        // setting up maple sim field
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
    GompeiLib.init(Constants.getMode(), Constants.TUNING_MODE, Constants.LOOP_PERIOD_SECONDS);

    switch (RobotConfig.ROBOT) {
      case V3_Horse, V3_Horse_SIM:
        robotContainer = new V3_HorseRobotContainer();
        break;
      default:
        robotContainer = new RobotContainer() {};
    }
  }

  @Override
  public void robotPeriodic() {
    // Refresh all registered CTRE status signals before any subsystem reads its inputs
    PhoenixUtil.refreshAll();
    VirtualSubsystem.periodicAll();
    robotContainer.robotPeriodic();
    Scheduler.getDefault().run();
  }

  @Override
  public void disabledInit() {
    Elastic.selectTab("Autonomous");
    Scheduler.getDefault().cancelAll();

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Elastic.selectTab("Autonomous");
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      Scheduler.getDefault().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Elastic.selectTab("Teleoperated");
    HubActivePeriod.initialize();
    if (autonomousCommand != null) {
      Scheduler.getDefault().cancel(autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void utilityInit() {
    Scheduler.getDefault().cancelAll();
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {}
}
