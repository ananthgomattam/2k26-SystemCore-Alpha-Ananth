package first.robot.v3_Horse;

import static org.wpilib.units.Units.RadiansPerSecond;

import edu.wpi.team190.gompeilib.io.gyro.interfaces.GyroIO;
import edu.wpi.team190.gompeilib.io.gyro.phoenix.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.phoenix.MotorIOTalonFX;
import edu.wpi.team190.gompeilib.io.motor.sim.MotorIOSim;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.robot.RobotContainer;
import edu.wpi.team190.gompeilib.robot.RobotMode;
import edu.wpi.team190.gompeilib.robot.mechanisms.extension.Extension;
import edu.wpi.team190.gompeilib.robot.mechanisms.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.robot.mechanisms.hood.GenericHood;
import edu.wpi.team190.gompeilib.robot.mechanisms.roller.GenericRoller;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveModuleHardware;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveBehaviors;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveSticks;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.SwerveDriveCommands;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.phoenix.SwerveModuleHardwareTalonFX;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.camera.CameraStaticLimelight;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIOLimelight;
import first.robot.Constants;
import first.robot.FieldConstants;
import first.robot.RobotConfig;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.CV2_input.XKeysInput;
import first.robot.v3_Horse.commands.V3_HorseCompositeCommands;
import first.robot.v3_Horse.commands.V3_HorseDriveCommands;
import first.robot.v3_Horse.commands.autonomous.V3_HorseAutoBackUpShoot;
import first.robot.v3_Horse.commands.autonomous.V3_HorseAutoDoubleDip;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntake;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.ExtensionState;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.RollerState;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloor;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloorConstants;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloorConstants.RollerFloorState;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooter;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooterConstants;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooterConstants.ShooterGoal;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringSubscriber;

public class V3_HorseRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private V3_HorseRollerFloor rollerFloor;
  private V3_HorseIntake intake;
  private V3_HorseShooter shooter;

  private Vision vision;

  private final LoggedNetworkChooser<Command> autoChooser;

  private final StringSubscriber selectedAutoName =
      NetworkTableInstance.getDefault()
          .getStringTopic("/Autonomous Modes/selected/value")
          .subscribe("");

  private final Map<String, Pose2d[]> autoTrajectories =
      Map.of(
          "Back up and shoot",
          V3_HorseAutoBackUpShoot.getTrajectory(),
          "Left Cheesy",
          V3_HorseAutoDoubleDip.getTrajectory("DOUBLE_DIP_1", "DOUBLE_DIP_2_CHEESY", false),
          "Right Cheesy",
          V3_HorseAutoDoubleDip.getTrajectory("DOUBLE_DIP_1", "DOUBLE_DIP_2_CHEESY", true),
          "Left Outliers",
          V3_HorseAutoDoubleDip.getTrajectory("DOUBLE_DIP_1", "DOUBLE_DIP_2_OUTLIERS", false),
          "Right Outliers",
          V3_HorseAutoDoubleDip.getTrajectory("DOUBLE_DIP_1", "DOUBLE_DIP_2_OUTLIERS", true));

  private String lastPublishedAuto = null;
  private boolean lastPublishedFlipped = false;

  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  @SuppressWarnings("unused")
  private final XKeysInput xkeys = new XKeysInput(1);

  private final CommandNiDsXboxController operator = new CommandNiDsXboxController(2);

  public V3_HorseRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V3_Horse:
          PhoenixOdometryThread odometryThread =
              PhoenixOdometryThread.getInstance(V3_HorseConstants.DRIVE_CONSTANTS);
          drive =
              new SwerveDrive(
                  V3_HorseConstants.DRIVE_CONSTANTS,
                  new GyroIOPigeon2(
                      V3_HorseConstants.DRIVE_CONSTANTS.gyroConfig(),
                      odometryThread,
                      V3_HorseRobotState::setHeadingUpdateTimestamp),
                  SwerveModuleHardwareTalonFX.create(
                      V3_HorseConstants.DRIVE_CONFIG.frontLeft(), odometryThread),
                  SwerveModuleHardwareTalonFX.create(
                      V3_HorseConstants.DRIVE_CONFIG.frontRight(), odometryThread),
                  SwerveModuleHardwareTalonFX.create(
                      V3_HorseConstants.DRIVE_CONFIG.backLeft(), odometryThread),
                  SwerveModuleHardwareTalonFX.create(
                      V3_HorseConstants.DRIVE_CONFIG.backRight(), odometryThread),
                  V3_HorseRobotState::getGlobalPose,
                  V3_HorseRobotState::resetPose);
          rollerFloor = new V3_HorseRollerFloor(MotorIOTalonFX.FACTORY);
          intake =
              new V3_HorseIntake(
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  () -> driver.getRightTriggerAxis());
          shooter =
              new V3_HorseShooter(
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  V3_HorseRobotState::isHopperRunning);
          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V3_HorseConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V3_HorseConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V3_HorseRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V3_HorseConstants.LIMELIGHT_LEFT_CONFIG),
                      V3_HorseConstants.LIMELIGHT_LEFT_CONFIG,
                      V3_HorseRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                      List.of()));

          break;
        case V3_Horse_SIM:
          drive =
              new SwerveDrive(
                  V3_HorseConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  SwerveModuleHardware.sim(
                      V3_HorseConstants.DRIVE_CONFIG.frontLeft(), V3_HorseConstants.DRIVE_CONFIG),
                  SwerveModuleHardware.sim(
                      V3_HorseConstants.DRIVE_CONFIG.frontRight(), V3_HorseConstants.DRIVE_CONFIG),
                  SwerveModuleHardware.sim(
                      V3_HorseConstants.DRIVE_CONFIG.backLeft(), V3_HorseConstants.DRIVE_CONFIG),
                  SwerveModuleHardware.sim(
                      V3_HorseConstants.DRIVE_CONFIG.backRight(), V3_HorseConstants.DRIVE_CONFIG),
                  V3_HorseRobotState::getGlobalPose,
                  V3_HorseRobotState::resetPose);
          rollerFloor =
              new V3_HorseRollerFloor(
                  MotorIOSim.factory(
                      GenericRoller.simMechanism(
                          V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS)));
          intake =
              new V3_HorseIntake(
                  MotorIOSim.factory(
                      GenericRoller.simMechanism(V3_HorseIntakeConstants.INTAKE_ROLLER_CONSTANTS)),
                  MotorIOSim.factory(
                      GenericRoller.simMechanism(V3_HorseIntakeConstants.KICKER_ROLLER_CONSTANTS)),
                  MotorIOSim.factory(
                      Extension.simMechanism(V3_HorseIntakeConstants.LEFT_EXTENSION_CONSTANTS)),
                  MotorIOSim.factory(
                      Extension.simMechanism(V3_HorseIntakeConstants.RIGHT_EXTENSION_CONSTANTS)),
                  () -> driver.getRightTriggerAxis());
          shooter =
              new V3_HorseShooter(
                  MotorIOSim.factory(
                      GenericFlywheel.simMechanism(V3_HorseShooterConstants.SHOOT_CONSTANTS)),
                  MotorIOSim.factory(
                      GenericHood.simMechanism(V3_HorseShooterConstants.HOOD_CONSTANTS)),
                  V3_HorseRobotState::isHopperRunning);
          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIO() {},
                      V3_HorseConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V3_HorseRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIO() {},
                      V3_HorseConstants.LIMELIGHT_LEFT_CONFIG,
                      V3_HorseRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                      List.of()));
          break;
        default:
      }
    }

    // Fall back to no-op IOs (e.g. in replay) so every subsystem always exists
    if (drive == null) {
      drive =
          new SwerveDrive(
              V3_HorseConstants.DRIVE_CONSTANTS,
              new GyroIO() {},
              SwerveModuleHardware.REPLAY,
              SwerveModuleHardware.REPLAY,
              SwerveModuleHardware.REPLAY,
              SwerveModuleHardware.REPLAY,
              V3_HorseRobotState::getGlobalPose,
              V3_HorseRobotState::resetPose);
    }
    if (rollerFloor == null) {
      rollerFloor = new V3_HorseRollerFloor(MotorIO.REPLAY_FACTORY);
    }
    if (intake == null) {
      intake =
          new V3_HorseIntake(
              MotorIO.REPLAY_FACTORY,
              MotorIO.REPLAY_FACTORY,
              MotorIO.REPLAY_FACTORY,
              MotorIO.REPLAY_FACTORY,
              () -> driver.getRightTriggerAxis());
    }
    if (shooter == null) {
      shooter =
          new V3_HorseShooter(
              MotorIO.REPLAY_FACTORY, MotorIO.REPLAY_FACTORY, V3_HorseRobotState::isHopperRunning);
    }

    if (vision == null) {
      vision =
          new Vision(
              () -> FieldConstants.tagLayoutType.getLayout(),
              new CameraStaticLimelight(
                  new CameraIO() {},
                  V3_HorseConstants.LIMELIGHT_SHOOTER_CONFIG,
                  V3_HorseRobotState::getHeading,
                  drive::getMeasuredChassisVelocities,
                  List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                  List.of()),
              new CameraStaticLimelight(
                  new CameraIO() {},
                  V3_HorseConstants.LIMELIGHT_LEFT_CONFIG,
                  V3_HorseRobotState::getHeading,
                  drive::getMeasuredChassisVelocities,
                  List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                  List.of()));
    }

    autoChooser = new LoggedNetworkChooser<>("Autonomous Modes");
    configureButtonBindings();
    configureAutos();
  }

  private void configureAutos() {

    Map<String, Command> markerCommands =
        Map.of(
            "INTAKE", V3_HorseCompositeCommands.intakeCollect(intake),
            "STOP_INTAKE", intake.setRollerState(RollerState.IDLE));

    autoChooser.add(
        "Back up and shoot",
        V3_HorseAutoBackUpShoot.getAutoRoutine(
            drive, shooter, rollerFloor, intake, markerCommands));

    autoChooser.add(
        "Left Cheesy",
        V3_HorseAutoDoubleDip.getAutoRoutine(
            drive,
            shooter,
            rollerFloor,
            intake,
            markerCommands,
            "DOUBLE_DIP_1",
            "DOUBLE_DIP_2_CHEESY",
            false,
            4,
            5));

    autoChooser.add(
        "Right Cheesy",
        V3_HorseAutoDoubleDip.getAutoRoutine(
            drive,
            shooter,
            rollerFloor,
            intake,
            markerCommands,
            "DOUBLE_DIP_1",
            "DOUBLE_DIP_2_CHEESY",
            true,
            4,
            5));

    autoChooser.add(
        "Left Outliers",
        V3_HorseAutoDoubleDip.getAutoRoutine(
            drive,
            shooter,
            rollerFloor,
            intake,
            markerCommands,
            "DOUBLE_DIP_1",
            "DOUBLE_DIP_2_OUTLIERS",
            false,
            4,
            5));

    autoChooser.add(
        "Right Outliers",
        V3_HorseAutoDoubleDip.getAutoRoutine(
            drive,
            shooter,
            rollerFloor,
            intake,
            markerCommands,
            "DOUBLE_DIP_1",
            "DOUBLE_DIP_2_OUTLIERS",
            true,
            4,
            5));

    autoChooser.addDefault(
        "Do Nothing", Command.noRequirements(coroutine -> {}).named("Auto: Do Nothing"));

    // Characterizations of the closed-loop mechanisms print each one's plant and
    // gains when they
    // finish
    autoChooser.add(
        "Characterize: Drive",
        SwerveDriveCommands.feedforwardCharacterization(
            drive, V3_HorseConstants.DRIVE_CHARACTERIZATION));
    autoChooser.add("Characterize: Hood", shooter.characterizeHood());
    autoChooser.add("Characterize: Flywheel (Torque)", shooter.characterizeFlywheel());
    autoChooser.add("Characterize: Flywheel (Voltage)", shooter.characterizeFlywheelVoltage());
    autoChooser.add("Characterize: Extensions", intake.characterizeExtensions());
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        SwerveDriveCommands.joystickDrive(
                drive,
                V3_HorseConstants.DRIVE_CONSTANTS,
                new DriveSticks(
                    () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()),
                V3_HorseRobotState::getHeading)
            .withDriverPerspective(() -> AllianceFlipUtil.apply(Rotation2d.ZERO))
            // Aim at hub or feed point
            .with(
                driver.rightBumper(),
                DriveBehaviors.faceHeading(
                    V3_HorseConstants.DRIVE_CONSTANTS, V3_HorseRobotState::getAimAngle))
            .with(
                driver.leftTrigger(),
                DriveBehaviors.faceHeading(
                    V3_HorseConstants.DRIVE_CONSTANTS, V3_HorseRobotState::getCardinalDirection))
            // School bus: face the direction of travel (lowest priority, toggled on right
            // stick)
            .with(
                V3_HorseDriveCommands.schoolBusActive(
                    () -> -driver.getLeftY(), () -> -driver.getLeftX()),
                DriveBehaviors.faceHeading(
                    V3_HorseConstants.DRIVE_CONSTANTS,
                    V3_HorseDriveCommands.schoolBusHeading(
                        () -> -driver.getLeftY(), () -> -driver.getLeftX())))
            .named("Drive: Joystick Drive"));

    driver.leftTrigger().onTrue(Command.noRequirements(coroutine -> {}).named("No Command"));

    driver
        .leftBumper()
        .whileTrue(V3_HorseCompositeCommands.intakeCollect(intake))
        .onFalse(intake.setRollerState(RollerState.IDLE));

    driver
        .rightBumper()
        .whileTrue(V3_HorseCompositeCommands.scoreOrFeedCommand(rollerFloor, shooter, intake))
        .onFalse(stopShooter());

    driver
        .rightTrigger(0.0)
        .onTrue(intake.setExtensionState(ExtensionState.MANUAL_EXTEND))
        .onFalse(intake.releaseManualExtend());

    driver
        .getHID()
        .povDown()
        .onTrue(
            V3_HorseCompositeCommands.resetHeading(
                V3_HorseRobotState::resetPose,
                () -> V3_HorseRobotState.getGlobalPose().getTranslation()));

    driver
        .y()
        .whileTrue(
            V3_HorseCompositeCommands.fixedShotCommand(
                rollerFloor, shooter, intake, V3_HorseRobotState.FixedShots.CORNER))
        .onFalse(stopShooter());

    driver
        .a()
        .whileTrue(
            V3_HorseCompositeCommands.fixedShotCommand(
                rollerFloor, shooter, intake, V3_HorseRobotState.FixedShots.FRONT_TOWER))
        .onFalse(stopShooter());

    driver
        .x()
        .whileTrue(
            V3_HorseCompositeCommands.fixedShotCommand(
                rollerFloor, shooter, intake, V3_HorseRobotState.FixedShots.SIDE_TOWER))
        .onFalse(stopShooter());

    driver.start().onTrue(shooter.setFlywheelVelocity(RadiansPerSecond.of(165)));

    driver.b().onTrue(V3_HorseCompositeCommands.intakeStow(intake));

    driver.rightStick().onTrue(V3_HorseDriveCommands.toggleSchoolBus());

    operator.leftBumper().onTrue(V3_HorseCompositeCommands.intakeStow(intake));

    operator
        .rightBumper()
        .whileTrue(V3_HorseCompositeCommands.intakeCollect(intake))
        .onFalse(intake.setRollerState(RollerState.IDLE));

    operator
        .leftTrigger()
        .whileTrue(intake.intakeRollers())
        .onFalse(intake.setRollerState(RollerState.IDLE));

    operator
        .rightTrigger()
        .whileTrue(intake.extakeRollers())
        .onFalse(intake.setRollerState(RollerState.IDLE));
    operator
        .getHID()
        .povUp()
        .whileTrue(
            Command.noRequirements(
                    corountine ->
                        rollerFloor.setOverrideRollerFloorVoltage(
                            V3_HorseRollerFloorConstants.ROLLER_FLOOR_SLOW_VOLTAGE))
                .named("RollerFloor: Slow Forward")
                .andThen(rollerFloor.setState(RollerFloorState.OVERRIDE))
                .withAutomaticName())
        .onFalse(rollerFloor.setState(RollerFloorState.STOP));

    operator
        .getHID()
        .povDown()
        .whileTrue(
            Command.noRequirements(
                    coroutine ->
                        rollerFloor.setOverrideRollerFloorVoltage(
                            V3_HorseRollerFloorConstants.ROLLER_FLOOR_RUN_VOLTAGE.times(-1)))
                .named("RollerFloor: Reverse")
                .andThen(rollerFloor.setState(RollerFloorState.OVERRIDE))
                .withAutomaticName())
        .onFalse(rollerFloor.setState(RollerFloorState.STOP));

    operator.getHID().povLeft().onTrue(shooter.resetHoodZero());

    operator.getHID().povRight().onTrue(intake.zeroExtensions());

    operator.b().onTrue(shooter.incrementFlywheelVelocity());
    operator.a().onTrue(shooter.decrementFlywheelVelocity());

    operator
        .x()
        .whileTrue(
            V3_HorseCompositeCommands.fixedShotCommand(
                rollerFloor, shooter, intake, V3_HorseRobotState.FixedShots.FEED))
        .onFalse(stopShooter());

    // operator.b().onTrue(shooter.incrementHoodAngle());
    // operator.a().onTrue(shooter.decrementHoodAngle());

    operator.back().onTrue(shooter.setGoal(ShooterGoal.STOW));
    operator.start().onTrue(intake.resetIntakeZeroPosition());

    operator.leftStick().whileTrue(intake.slowMoveIn()).onFalse(intake.stopExtensions());
    operator.rightStick().whileTrue(intake.slowMoveOut()).onFalse(intake.stopExtensions());
  }

  private Command stopShooter() {
    return V3_HorseCompositeCommands.stopShooter(rollerFloor, shooter);
  }

  @Override
  public void robotPeriodic() {
    V3_HorseRobotState.periodic(
        drive.getRawGyroRotation(), drive.getYawVelocity(), drive.getModulePositions(), drive);
    Logger.recordOutput(
        "Mechanism3D",
        V3_HorseMechanism3D.getPoses(intake.getExtensionPosition(), shooter.getHoodAngle()));

    String autoName = selectedAutoName.get();
    boolean flipped = AllianceFlipUtil.shouldFlip();

    Logger.recordOutput("Drive/School Bus/Enabled", V3_HorseDriveCommands.isSchoolBusEnabled());
    Logger.recordOutput("Drive/School Bus/Deadband", V3_HorseDriveCommands.getSchoolBusDeadband());

    if (!autoName.equals(lastPublishedAuto) || flipped != lastPublishedFlipped) {
      Pose2d[] trajectory = autoTrajectories.getOrDefault(autoName, new Pose2d[0]);
      if (flipped) {
        trajectory = Arrays.stream(trajectory).map(AllianceFlipUtil::apply).toArray(Pose2d[]::new);
      }
      V3_HorseRobotState.setAutoTrajectory(trajectory);

      lastPublishedAuto = autoName;
      lastPublishedFlipped = flipped;
    }
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
