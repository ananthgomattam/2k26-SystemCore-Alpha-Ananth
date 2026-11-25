package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.team190.gompeilib.auto.AutoPath;
import edu.wpi.team190.gompeilib.auto.PathMarker;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig.Leg;
import edu.wpi.team190.gompeilib.characterization.CharacterizationRoutine;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Sample;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Distance;

/** Robot-agnostic swerve drive commands. */
public final class SwerveDriveCommands {
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private SwerveDriveCommands() {}

  /**
   * Starts a joystick drive. Bind behaviors with {@link JoystickDrive.Builder#with}, then {@link
   * JoystickDrive.Builder#named}.
   *
   * @param drive the drive
   * @param constants the drive's constants, for the driver deadband
   * @param sticks the driver's sticks
   * @param heading the robot's field heading
   */
  public static JoystickDrive.Builder joystickDrive(
      SwerveDrive drive,
      SwerveDriveConstants constants,
      DriveSticks sticks,
      Supplier<Rotation2d> heading) {
    return new JoystickDrive.Builder(drive, sticks, constants.driverDeadband, heading);
  }

  /** Turns the robot in place to a field heading, holding it there until interrupted. */
  public static Command rotateToAngle(
      SwerveDrive drive,
      SwerveDriveConstants constants,
      Supplier<Rotation2d> heading,
      Supplier<Rotation2d> target) {
    return joystickDrive(drive, constants, DriveSticks.NONE, heading)
        .with(() -> true, DriveBehaviors.faceHeading(constants, target))
        .named("Rotate To Angle");
  }

  /** Stops the drive with its modules in an X, holding until interrupted. */
  public static Command stop(SwerveDrive drive) {
    return drive.runRepeatedly(drive::stopWithX).named("X Lock");
  }

  /**
   * Characterizes the drive motors with the modules facing forward, then prints the drive's plant
   * and gains per wheel rotation. Characterize in the unit the drive's closed loop outputs: {@link
   * CharacterizationConfig#current()} for a torque-current loop, {@link
   * CharacterizationConfig#voltage()} otherwise. Every leg drives the robot straight until it times
   * out, so give it room and keep the timeouts short.
   *
   * @param drive the drive
   * @param config the ramp, step, timeouts, settle time and legs
   * @param <U> the output unit
   * @return the command
   */
  public static <U extends Unit> Command feedforwardCharacterization(
      SwerveDrive drive, CharacterizationConfig<U> config) {
    return new CharacterizationRoutine<>(
            config,
            drive,
            List.of(new DriveTarget<U>(drive, OutputType.of(config.getOutputUnit()).orElse(null))))
        .command();
  }

  /**
   * The drive motors as one characterization target: the four modules driven together, sampled as
   * their average.
   *
   * @param drive the drive
   * @param output the output the motors are driven in
   * @param <U> the output unit
   */
  record DriveTarget<U extends Unit>(SwerveDrive drive, OutputType output)
      implements CharacterizationRoutine.Target<U> {
    @Override
    public String topic() {
      return "Drive";
    }

    @Override
    public void idle() {
      drive.stop();
    }

    @Override
    public void drive(Measure<U> output) {
      double value = output.baseUnitMagnitude();
      drive.runCharacterization(
          this.output == OutputType.CURRENT
              ? new OpenLoopRequest.CurrentOutput(Amps.of(value))
              : new OpenLoopRequest.VoltageOutput(Volts.of(value)));
    }

    @Override
    public Sample sample(OutputType effort) {
      return drive.getFFCharacterizationSample(effort);
    }

    @Override
    public boolean atLimit(Leg leg) {
      return false;
    }

    @Override
    public Optional<GravityType> gravity() {
      return Optional.empty();
    }

    @Override
    public Optional<Distance> distancePerRadian() {
      return Optional.empty();
    }
  }

  /**
   * Spins the robot in place and, when interrupted, prints the wheel radius that matches the gyro's
   * rotation to the wheels' travel.
   */
  public static Command wheelRadiusCharacterization(
      SwerveDrive drive, SwerveDriveConstants constants) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    Timer timer = new Timer();
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return drive
        .run(
            coroutine -> {
              limiter.reset(0.0);
              timer.restart();
              state.measuring = false;
              while (true) {
                drive.runVelocity(
                    new ChassisVelocities(0.0, 0.0, limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY)));
                if (!state.measuring && timer.hasElapsed(1.0)) {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRawGyroRotation();
                  state.gyroDelta = 0.0;
                  state.measuring = true;
                } else if (state.measuring) {
                  Rotation2d rotation = drive.getRawGyroRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                }

                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              if (state.measuring) {
                printWheelRadius(state, drive, constants.driveConfig.driveBaseRadius());
              }
            })
        .named("Wheel Radius Characterization");
  }

  private static void printWheelRadius(
      WheelRadiusCharacterizationState state, SwerveDrive drive, double driveBaseRadius) {
    double[] positions = drive.getWheelRadiusCharacterizationPositions();
    double wheelDelta = 0.0;
    for (int i = 0; i < 4; i++) {
      wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
    }
    double wheelRadius = (state.gyroDelta * driveBaseRadius) / wheelDelta;

    NumberFormat formatter = new DecimalFormat("#0.000");
    System.out.println("********** Wheel Radius Characterization Results **********");
    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
    System.out.println(
        "\tWheel Radius: "
            + formatter.format(wheelRadius)
            + " meters, "
            + formatter.format(Units.metersToInches(wheelRadius))
            + " inches");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.ZERO;
    double gyroDelta = 0.0;
    boolean measuring = false;
  }

  /**
   * Returns the time at which a position is crossed on a path
   *
   * @param path The Auto Path
   * @param trajectory The Auto Trajectory (Time Based)
   * @param position The position we are trying to find the time of
   * @return The time
   */
  private static double timeAt(
      PathPlannerPath path, PathPlannerTrajectory trajectory, double position) {
    List<PathPoint> pathPoints = path.getAllPathPoints();
    int bestIndex = 0;
    double smallestDiff = Math.abs(pathPoints.get(0).waypointRelativePos - position);
    for (int i = 0; i < pathPoints.size(); i++) {
      double diff = Math.abs(pathPoints.get(i).waypointRelativePos - position);
      if (diff < smallestDiff) {
        smallestDiff = diff;
        bestIndex = i;
      }
    }
    return trajectory.getState(bestIndex).timeSeconds;
  }

  /**
   * Folows an auto path from PathPlanner
   *
   * @param drive The drive subsystem
   * @param autoPath The path to follow
   * @param markerCommands The PathPlanner named commands
   * @return A coroutine for the auto path follow
   */
  public static Command followPath(
      SwerveDrive drive, AutoPath autoPath, Map<String, Command> markerCommands) {
    return drive
        .run(
            coroutine -> {
              PathPlannerPath path =
                  drive.shouldFlipPath() ? autoPath.path().flipPath() : autoPath.path();
              PPHolonomicDriveController controller = drive.getPathController();
              Pose2d startPose = drive.getPose();
              ChassisVelocities startSpeeds = drive.getMeasuredChassisVelocities();
              controller.reset(startPose, startSpeeds);
              PathPlannerTrajectory trajectory =
                  path.generateTrajectory(
                      startSpeeds, startPose.getRotation(), drive.getPathConfig());

              List<PathMarker> waiting = new ArrayList<>(autoPath.markers());
              Map<PathMarker, Command> zones = new HashMap<>();

              Timer timer = new Timer();
              timer.start();
              while (!timer.hasElapsed(trajectory.getTotalTimeSeconds())) {
                double time = timer.get();
                drive.runVelocity(
                    controller.calculateRobotRelativeSpeeds(
                        drive.getPose(), trajectory.sample(time)));

                for (PathMarker marker : new ArrayList<>(waiting)) {
                  if (time >= timeAt(path, trajectory, marker.start())) {
                    if (markerCommands.containsKey(marker.command())) {
                      coroutine.fork(markerCommands.get(marker.command()));
                      if (marker.isZone()) {
                        zones.put(marker, markerCommands.get(marker.command()));
                      }
                    }
                    waiting.remove(marker);
                  }
                }

                for (PathMarker marker : new ArrayList<>(zones.keySet())) {
                  if (time >= timeAt(path, trajectory, marker.end())) {
                    coroutine.scheduler().cancel(zones.get(marker));
                    zones.remove(marker);
                  }
                }
                coroutine.yield();
              }

              if (path.getGoalEndState().velocityMPS() < 0.1) {
                drive.stop();
              }
            })
        .named("Drive: Follow Path");
  }
}
