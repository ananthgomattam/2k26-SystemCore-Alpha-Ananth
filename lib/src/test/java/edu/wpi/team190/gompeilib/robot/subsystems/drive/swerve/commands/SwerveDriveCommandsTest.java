package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveCommandsTestSupport.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.auto.AutoPath;
import edu.wpi.team190.gompeilib.auto.PathMarker;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig.Leg;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Sample;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.SimHooks;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.VoltageUnit;

public class SwerveDriveCommandsTest extends GompeiLibTestBase {
  private static final double LOOP_SECONDS = 0.02;

  private MockedStatic<Logger> logger;
  private SwerveDrive drive;
  private SwerveDriveConstants constants;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    drive = DriveCommandsTestSupport.drive();
    constants = DriveCommandsTestSupport.constants();
  }

  @AfterEach
  public void tearDown() {
    logger.close();
    SimHooks.resumeTiming();
  }

  @Test
  public void testJoystickDriveUsesTheDriverDeadband() {
    JoystickDrive command =
        SwerveDriveCommands.joystickDrive(
                drive,
                constants,
                new DriveSticks(() -> 0.05, () -> 0.0, () -> 0.0),
                () -> Rotation2d.ZERO)
            .named("Joystick Drive");
    start(command);
    verify(drive).runVelocity(new ChassisVelocities(0.0, 0.0, 0.0));
  }

  @Test
  public void testRotatesInPlaceToTheTarget() {
    Command command =
        SwerveDriveCommands.rotateToAngle(
            drive, constants, () -> Rotation2d.ZERO, () -> Rotation2d.fromDegrees(90.0));
    assertTrue(command.requirements().contains(drive));

    start(command);
    ArgumentCaptor<ChassisVelocities> captor = ArgumentCaptor.forClass(ChassisVelocities.class);
    verify(drive).runVelocity(captor.capture());
    assertEquals(0.0, captor.getValue().vx, TOLERANCE);
    assertEquals(0.0, captor.getValue().vy, TOLERANCE);
    assertTrue(captor.getValue().omega > 0.0);
  }

  @Test
  public void testStopHoldsTheModulesInAnX() {
    Command command = SwerveDriveCommands.stop(drive);
    assertTrue(command.requirements().contains(drive));
    start(command);
    verify(drive).stopWithX();
    assertTrue(isRunning(command));
  }

  @Test
  public void testFeedforwardCharacterizationRunsTheRoutineOnTheDrive() {
    when(drive.getName()).thenReturn("SwerveDrive");
    Command command =
        SwerveDriveCommands.feedforwardCharacterization(
            drive,
            CharacterizationConfig.voltage()
                .withRampRate(VoltsPerSecond.ofNative(1))
                .withStepOutput(Volts.of(4))
                .withQuasistaticTimeout(Seconds.of(2))
                .withDynamicTimeout(Seconds.of(1))
                .build());
    assertEquals("SwerveDrive: Characterization", command.name());
    assertTrue(command.requirements().contains(drive));

    assertThrows(
        IllegalArgumentException.class,
        () ->
            SwerveDriveCommands.feedforwardCharacterization(
                drive,
                CharacterizationConfig.<DimensionlessUnit>builder()
                    .withOutputUnit(Percent)
                    .withRampRate(PercentPerSecond.ofNative(1))
                    .withStepOutput(Percent.of(10))
                    .withQuasistaticTimeout(Seconds.of(2))
                    .withDynamicTimeout(Seconds.of(1))
                    .build()));
  }

  @Test
  public void testDriveTargetDrivesEveryModuleInItsOutput() {
    SwerveDriveCommands.DriveTarget<VoltageUnit> volts =
        new SwerveDriveCommands.DriveTarget<>(drive, OutputType.VOLTAGE);
    volts.drive(Volts.of(3));
    verify(drive).runCharacterization(new OpenLoopRequest.VoltageOutput(Volts.of(3.0)));

    SwerveDriveCommands.DriveTarget<CurrentUnit> amps =
        new SwerveDriveCommands.DriveTarget<>(drive, OutputType.CURRENT);
    amps.drive(Amps.of(20));
    verify(drive).runCharacterization(new OpenLoopRequest.CurrentOutput(Amps.of(20.0)));

    assertEquals("Drive", volts.topic());
    volts.idle();
    verify(drive).stop();
    Sample sample = new Sample(1, 2, 3, 4, 5);
    when(drive.getFFCharacterizationSample(OutputType.CURRENT)).thenReturn(sample);
    assertSame(sample, amps.sample(OutputType.CURRENT));
    assertFalse(volts.atLimit(Leg.DYNAMIC_FORWARD));
    assertTrue(volts.gravity().isEmpty());
    assertTrue(volts.distancePerRadian().isEmpty());
  }

  @Test
  public void testWheelRadiusCharacterizationMatchesTheGyroToTheWheels() {
    double driveBaseRadius = constants.driveConfig.driveBaseRadius();
    // Over the measurement, the gyro turns 1 rad and each wheel turns 20 rad
    when(drive.getWheelRadiusCharacterizationPositions())
        .thenReturn(new double[] {0.0, 0.0, 0.0, 0.0}, new double[] {20.0, -20.0, 20.0, -20.0});
    when(drive.getRawGyroRotation())
        .thenReturn(Rotation2d.ZERO, Rotation2d.fromRadians(0.5), Rotation2d.fromRadians(1.0));

    Command command = SwerveDriveCommands.wheelRadiusCharacterization(drive, constants);
    assertTrue(command.requirements().contains(drive));

    SimHooks.pauseTiming();
    start(command);
    // Spin until the modules have oriented and measurement has started
    for (int i = 0; i < 60; i++) {
      runScheduler();
      SimHooks.stepTiming(LOOP_SECONDS);
    }
    ArgumentCaptor<ChassisVelocities> captor = ArgumentCaptor.forClass(ChassisVelocities.class);
    verify(drive, atLeastOnce()).runVelocity(captor.capture());
    ChassisVelocities last = captor.getValue();
    assertEquals(0.0, last.vx, TOLERANCE);
    assertTrue(last.omega > 0.0 && last.omega <= 0.25);

    PrintStream original = System.out;
    ByteArrayOutputStream printed = new ByteArrayOutputStream();
    System.setOut(new PrintStream(printed));
    try {
      scheduler.cancel(command);
    } finally {
      System.setOut(original);
    }
    double expectedRadius = (1.0 * driveBaseRadius) / 20.0;
    assertTrue(
        printed.toString().contains(String.format("%.3f meters", expectedRadius)),
        printed.toString());
  }

  private static AutoPath straightPath(List<PathMarker> markers) {
    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(0.0, 0.0, Rotation2d.ZERO), new Pose2d(2.0, 0.0, Rotation2d.ZERO)),
            new PathConstraints(2.0, 2.0, 2.0 * Math.PI, 4.0 * Math.PI),
            null,
            new GoalEndState(0.0, Rotation2d.ZERO));
    return new AutoPath(path, markers);
  }

  private static Command forever(String name) {
    return Command.noRequirements(
            coroutine -> {
              while (true) {
                coroutine.yield();
              }
            })
        .named(name);
  }

  private void stubPathFollowing() {
    when(drive.getPose()).thenReturn(new Pose2d());
    when(drive.getPathController())
        .thenReturn(new PPHolonomicDriveController(new PIDConstants(1.0), new PIDConstants(1.0)));
    when(drive.getPathConfig())
        .thenReturn(
            new RobotConfig(
                50.0,
                6.0,
                new ModuleConfig(0.05, 4.0, 1.2, DCMotor.getKrakenX60(1), 6.75, 60.0, 1),
                new Translation2d(0.3, 0.3),
                new Translation2d(0.3, -0.3),
                new Translation2d(-0.3, 0.3),
                new Translation2d(-0.3, -0.3)));
    when(drive.shouldFlipPath()).thenReturn(false);
  }

  private boolean runUntilFinished(Command command, double seconds) {
    for (double time = 0.0; time < seconds; time += LOOP_SECONDS) {
      SimHooks.stepTiming(LOOP_SECONDS);
      runScheduler();
      if (!isRunning(command)) {
        return true;
      }
    }
    return false;
  }

  @Test
  public void testFollowPathDrivesThePathAndStopsAtTheEnd() {
    stubPathFollowing();
    Command command = SwerveDriveCommands.followPath(drive, straightPath(List.of()), Map.of());
    assertTrue(command.requirements().contains(drive));

    SimHooks.pauseTiming();
    start(command);
    assertTrue(runUntilFinished(command, 10.0));
    verify(drive, atLeastOnce()).runVelocity(argThat(speeds -> speeds.vx > 0.1));
    verify(drive).stop();
  }

  @Test
  public void testFollowPathStartsMarkersAndEndsZones() {
    stubPathFollowing();
    Command zone = forever("Zone");
    Command point = forever("Point");
    Command command =
        SwerveDriveCommands.followPath(
            drive,
            straightPath(
                List.of(new PathMarker("ZONE", 0.2, 0.5), new PathMarker("POINT", 0.7, -1.0))),
            Map.of("ZONE", zone, "POINT", point));

    SimHooks.pauseTiming();
    start(command);
    assertFalse(isRunning(zone));
    assertFalse(isRunning(point));

    List<String> events = new ArrayList<>();
    boolean zoneWasRunning = false;
    boolean pointWasRunning = false;
    for (int i = 0; i < 500 && isRunning(command); i++) {
      SimHooks.stepTiming(LOOP_SECONDS);
      runScheduler();
      boolean zoneRunning = isRunning(zone);
      boolean pointRunning = isRunning(point);
      if (zoneRunning && !zoneWasRunning) {
        events.add("zone start");
      }
      if (!zoneRunning && zoneWasRunning && isRunning(command)) {
        events.add("zone end");
      }
      if (pointRunning && !pointWasRunning) {
        events.add("point start");
      }
      zoneWasRunning = zoneRunning;
      pointWasRunning = pointRunning;
    }

    assertFalse(isRunning(command));
    assertEquals(List.of("zone start", "zone end", "point start"), events);
    assertFalse(isRunning(point));
  }

  @Test
  public void testFollowPathSkipsMarkersWithNoCommand() {
    stubPathFollowing();
    Command command =
        SwerveDriveCommands.followPath(
            drive, straightPath(List.of(new PathMarker("MISSING", 0.5, -1.0))), Map.of());

    SimHooks.pauseTiming();
    start(command);
    assertTrue(runUntilFinished(command, 10.0));
  }
}
