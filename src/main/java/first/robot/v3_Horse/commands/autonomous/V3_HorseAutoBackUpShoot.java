package first.robot.v3_Horse.commands.autonomous;

import edu.wpi.team190.gompeilib.auto.AutoPath;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.SwerveDriveCommands;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.Elastic;
import first.robot.v3_Horse.V3_HorseRobotState;
import first.robot.v3_Horse.commands.V3_HorseCompositeCommands;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntake;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloor;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooter;
import java.util.Map;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;

public class V3_HorseAutoBackUpShoot {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      V3_HorseShooter shooter,
      V3_HorseRollerFloor rollerFloor,
      V3_HorseIntake intake,
      Map<String, Command> markerCommands) {

    AutoPath BACK_UP_SHOOT;

    try {
      BACK_UP_SHOOT = AutoPath.load("BACK_UP_SHOOT");

      return Command.sequence(
              Command.noRequirements(
                      coroutine ->
                          V3_HorseRobotState.resetPose(
                              AllianceFlipUtil.apply(
                                  BACK_UP_SHOOT.path().getStartingHolonomicPose().get())))
                  .named("Auto: Reset Pose"),
              SwerveDriveCommands.followPath(drive, BACK_UP_SHOOT, markerCommands),
              drive.run(coroutine -> drive.stop()).named("Drive: Stop"),
              V3_HorseCompositeCommands.autoScoreOrFeed(drive, rollerFloor, shooter, intake, 10),
              V3_HorseCompositeCommands.stopShooter(rollerFloor, shooter))
          .named("Auto: Back Up And Shoot");

    } catch (Exception e) {
      e.printStackTrace();
      return Command.noRequirements(
              coroutine ->
                  Elastic.sendNotification(
                      new Elastic.Notification(
                          Elastic.NotificationLevel.ERROR,
                          "Failed to load auto path",
                          e.getMessage())))
          .named("Elastic: Failed To Load Auto Path");
    }
  }

  public static Pose2d[] getTrajectory() {
    try {
      return AutoPath.load("BACK_UP_SHOOT").path().getPathPoses().toArray(new Pose2d[0]);
    } catch (Exception e) {
      return new Pose2d[0];
    }
  }
}
