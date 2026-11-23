package first.robot.v3_Horse.commands;

import static org.wpilib.units.Units.Seconds;

import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.v3_Horse.V3_HorseConstants;
import first.robot.v3_Horse.V3_HorseRobotState;
import first.robot.v3_Horse.V3_HorseRobotState.FixedShots;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntake;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.ExtensionState;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.RollerState;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloor;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloorConstants;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloorConstants.RollerFloorState;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooter;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooterConstants.ShooterGoal;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class V3_HorseCompositeCommands {

  public static Command resetHeading(
      Consumer<Pose2d> resetHeadingConsumer, Supplier<Translation2d> currentRobotTranslation) {
    return Command.noRequirements(
            coroutine -> {
              resetHeadingConsumer.accept(
                  new Pose2d(
                      currentRobotTranslation.get(), AllianceFlipUtil.apply(new Rotation2d())));
            })
        .named("Drive: Reset Heading");
  }

  public static Command updateCurrentLimits(
      SwerveDrive drive, double driveCurrentLimit, double turnCurrentLimit) {
    return Command.noRequirements(
            coroutine -> {
              drive.updateCurrentLimits(driveCurrentLimit, turnCurrentLimit);
            })
        .named("Drive: Update Current Limits");
  }

  public static Command deployIntake(V3_HorseIntake intake) {
    return Command.parallel(
            intake.setExtensionState(ExtensionState.INTAKE),
            intake.setRollerState(RollerState.IDLE))
        .named("Intake: Deploy");
  }

  public static Command intakeCollect(V3_HorseIntake intake) {
    return Command.parallel(
            intake.setExtensionState(ExtensionState.INTAKE),
            intake.setRollerState(RollerState.INTAKE))
        .named("Intake: Collect");
  }

  public static Command intakeStow(V3_HorseIntake intake) {
    return Command.parallel(
            intake.setExtensionState(ExtensionState.STOW), intake.setRollerState(RollerState.IDLE))
        .named("Intake: Stow");
  }

  public static Command intakeSpit(V3_HorseIntake intake, V3_HorseRollerFloor rollerfloor) {
    return Command.parallel(
            intake.setExtensionState(ExtensionState.INTAKE),
            intake.setRollerState(RollerState.EXTAKE),
            rollerfloor.setState(RollerFloorState.OVERRIDE),
            Command.noRequirements(
                    coroutine ->
                        rollerfloor.setOverrideRollerFloorVoltage(
                            V3_HorseRollerFloorConstants.ROLLER_FLOOR_RUN_VOLTAGE.times(-1)))
                .named("Roller Floor: Run Backwards"))
        .named("Intake: Spit");
  }

  public static Command autoScoreOrFeed(
      SwerveDrive drive,
      V3_HorseRollerFloor rollerFloor,
      V3_HorseShooter shooter,
      V3_HorseIntake intake,
      double seconds) {
    return Command.race(
            V3_HorseDriveCommands.aimAtHub(drive, V3_HorseConstants.DRIVE_CONSTANTS),
            scoreOrFeedCommand(rollerFloor, shooter, intake),
            Command.waitFor(Seconds.of(seconds)).named("Wait " + seconds + " Seconds"))
        .named("Score Or Feed Command (Auto)");
  }

  /**
   * Spins up for score or feed, thxen runs the roller floor once the shooter and heading are ready.
   */
  public static Command scoreOrFeedCommand(
      V3_HorseRollerFloor rollerFloor, V3_HorseShooter shooter, V3_HorseIntake intake) {
    return Command.parallel(
            shooter.setGoal(
                () ->
                    (V3_HorseRobotState.isInAllianceZone() ? ShooterGoal.SCORE : ShooterGoal.FEED)),
            Command.sequence(
                    rollerFloor.setState(RollerFloorState.SPIN_UP),
                    Command.waitUntil(() -> shooter.atGoal() && isAimedForShot())
                        .named("Shooter: Wait Until Ready To Shoot"),
                    rollerFloor.setState(RollerFloorState.RUN),
                    intake.squishIntake())
                .withAutomaticName())
        .named("Score Or Feed Command");
  }

  private static boolean isAimedForShot() {
    return V3_HorseDriveCommands.atAngle(
        V3_HorseRobotState.getHeading(), V3_HorseRobotState.getAimAngle());
  }

  public static Command stopShooter(V3_HorseRollerFloor rollerFloor, V3_HorseShooter shooter) {
    return Command.parallel(
            shooter.setGoal(ShooterGoal.STOP), rollerFloor.setState(RollerFloorState.STOP))
        .named("Stop Shooter");
  }

  /**
   * Holds the fixed robot heading while spinning up, then runs the roller floor once the shooter
   * and heading are ready.
   */
  public static Command fixedShotCommand(
      V3_HorseRollerFloor rollerFloor,
      V3_HorseShooter shooter,
      V3_HorseIntake intake,
      FixedShots fixedShot) {
    return Command.parallel(
            Command.sequence(
                    shooter.runFixedShot(fixedShot),
                    rollerFloor.setState(RollerFloorState.SPIN_UP),
                    shooter.waitUntilAtGoal(),
                    rollerFloor.setState(RollerFloorState.RUN),
                    intake.squishIntake())
                .withAutomaticName())
        .named("Run Fixed Shot: " + fixedShot.toString());
  }
}
