package first.robot.v3_Horse.commands.autonomous;

import static org.wpilib.units.Units.RadiansPerSecond;

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
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;

public class V3_HorseAutoDoubleDip {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      V3_HorseShooter shooter,
      V3_HorseRollerFloor rollerFloor,
      V3_HorseIntake intake,
      Map<String, Command> markerCommands,
      String path1Name,
      String path2Name,
      boolean mirror,
      double shot1Seconds,
      double shot2Seconds) {

    AutoPath DOUBLE_DIP_1;
    AutoPath DOUBLE_DIP_2;

    try {

      DOUBLE_DIP_1 = mirror ? AutoPath.load(path1Name).mirror() : AutoPath.load(path1Name);
      DOUBLE_DIP_2 = mirror ? AutoPath.load(path2Name).mirror() : AutoPath.load(path2Name);

      return Command.sequence(
              V3_HorseCompositeCommands.deployIntake(intake),
              shooter.setFlywheelVelocity(RadiansPerSecond.of(165)),
              Command.noRequirements(
                      coroutine ->
                          V3_HorseRobotState.resetPose(
                              AllianceFlipUtil.apply(
                                  DOUBLE_DIP_1.path().getStartingHolonomicPose().get())))
                  .named("Auto: Reset Pose"),
              SwerveDriveCommands.followPath(drive, DOUBLE_DIP_1, markerCommands),
              drive.run(coroutine -> drive.stop()).named("Drive: Stop"),
              V3_HorseCompositeCommands.autoScoreOrFeed(
                  drive, rollerFloor, shooter, intake, shot1Seconds),
              V3_HorseCompositeCommands.stopShooter(rollerFloor, shooter),
              V3_HorseCompositeCommands.deployIntake(intake),
              shooter.setFlywheelVelocity(RadiansPerSecond.of(165)),
              SwerveDriveCommands.followPath(drive, DOUBLE_DIP_2, markerCommands),
              drive.run(coroutine -> drive.stop()).named("Drive: Stop"),
              V3_HorseCompositeCommands.autoScoreOrFeed(
                  drive, rollerFloor, shooter, intake, shot2Seconds),
              V3_HorseCompositeCommands.stopShooter(rollerFloor, shooter))
          .named("Auto: Double Dip");
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

  public static Pose2d[] getTrajectory(String path1Name, String path2Name, boolean mirror) {
    try {
      AutoPath DOUBLE_DIP_1 = mirror ? AutoPath.load(path1Name).mirror() : AutoPath.load(path1Name);
      AutoPath DOUBLE_DIP_2 = mirror ? AutoPath.load(path2Name).mirror() : AutoPath.load(path2Name);

      List<Pose2d> poses = new ArrayList<>(DOUBLE_DIP_1.path().getPathPoses());
      poses.addAll(DOUBLE_DIP_2.path().getPathPoses());
      return poses.toArray(new Pose2d[0]);
    } catch (Exception e) {
      return new Pose2d[0];
    }
  }
}
