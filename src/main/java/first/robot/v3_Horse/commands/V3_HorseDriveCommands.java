package first.robot.v3_Horse.commands;

import static org.wpilib.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants.AutoAlignConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.SwerveDriveCommands;
import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import first.robot.FieldConstants;
import first.robot.util.AllianceFlipUtil;
import first.robot.v3_Horse.V3_HorseRobotState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.tunable.Tunable;

/**
 * This robot's drive commands, built on GompeiLib's {@link SwerveDriveCommands}. Joystick driving
 * is bound in the robot container.
 */
public final class V3_HorseDriveCommands {

  @Getter private static double slowFactor = 0.5;

  @Getter private static boolean schoolBusEnabled = false;
  private static final Tunable<Double> schoolBusDeadband =
      GompeiTunables.value("Drive/School Bus/Deadband", 0.3);

  /**
   * Whether school bus drive should turn the robot this loop: it's toggled on and the left stick is
   * pushed past the school bus deadband.
   */
  public static BooleanSupplier schoolBusActive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return () ->
        schoolBusEnabled
            && Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                > schoolBusDeadband.get();
  }

  /** The field heading the left stick is pushing toward, for school bus drive to face. */
  public static Supplier<Rotation2d> schoolBusHeading(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return () ->
        AllianceFlipUtil.apply(new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()));
  }

  public static Command toggleSchoolBus() {
    return Command.noRequirements(coroutine -> schoolBusEnabled = !schoolBusEnabled)
        .named("Drive: School Bus Toggled");
  }

  public static double getSchoolBusDeadband() {
    return schoolBusDeadband.get();
  }

  public static Command incrementSlowFactor() {
    return Command.noRequirements(coroutine -> slowFactor = Math.min(slowFactor + 0.05, 1.0))
        .named("Drive: Increment Slow Factor");
  }

  public static Command decrementSlowFactor() {
    return Command.noRequirements(coroutine -> slowFactor = Math.max(slowFactor - 0.05, 0.0))
        .named("Drive: Decrement Slow Factor");
  }

  public static boolean atAngle(Rotation2d currentRotation, Rotation2d targetRotation) {
    return Math.abs(currentRotation.minus(targetRotation).getRadians())
        <= Units.degreesToRadians(2.0);
  }

  public static Command autoAlignPoseCommand(
      SwerveDrive drive,
      Supplier<Pose2d> robotPoseSupplier,
      Pose2d targetPose,
      AutoAlignConstants constants) {
    return new V3_HorseAutoAlignCommands(
        drive,
        targetPose,
        () -> true,
        robotPoseSupplier,
        constants,
        // Must be finite: alpha-7 TrapezoidProfile computes inf * 0 = NaN otherwise
        constants.xConstraints().maxAcceleration().get().in(MetersPerSecondPerSecond));
  }

  public static Command autoAlignTowerCommand(
      SwerveDrive drive, Supplier<Pose2d> robotPoseSupplier, AutoAlignConstants constants) {
    return autoAlignPoseCommand(
        drive,
        robotPoseSupplier,
        new Pose2d(FieldConstants.Tower.centerPoint, new Rotation2d()),
        constants);
  }

  /** Turns in place so the shooter faces the hub or the nearest feed point. */
  public static Command aimAtHub(SwerveDrive drive, SwerveDriveConstants driveConstants) {
    return SwerveDriveCommands.rotateToAngle(
        drive, driveConstants, V3_HorseRobotState::getHeading, V3_HorseRobotState::getAimAngle);
  }

  /** TODO: v3 path following. PathPlanner's AutoBuilder is Commands v2. Does nothing yet. */
  public static Command followPath(SwerveDrive drive, PathPlannerPath path) {
    return drive
        .run(coroutine -> System.out.println("WARNING: path following not ported to v3 yet"))
        .named("Drive: Follow Path (not ported)");
  }
}
