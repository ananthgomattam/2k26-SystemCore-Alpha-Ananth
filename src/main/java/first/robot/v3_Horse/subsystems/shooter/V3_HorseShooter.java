package first.robot.v3_Horse.subsystems.shooter;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.Feedforward;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.mechanisms.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.robot.mechanisms.hood.GenericHood;
import first.robot.v3_Horse.V3_HorseRobotState;
import first.robot.v3_Horse.V3_HorseRobotState.FixedShotParameters;
import first.robot.v3_Horse.V3_HorseRobotState.FixedShots;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooterConstants.ShooterGoal;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.util.Units;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class V3_HorseShooter implements Mechanism {

  private final GenericFlywheel flywheel;
  private ShooterGoal shooterGoal;
  private boolean characterizing = false;
  // Goal most recently pushed to the flywheel and hood in periodic
  private ShooterGoal appliedGoal;

  private final GenericHood hood;

  private final Trigger flywheelShootingTrigger;
  private final Trigger flywheelFeedingTrigger;

  private final Trigger hoodFeedingTrigger;

  private FixedShotParameters fixedShotParameters;

  private final BooleanSupplier isHopperRunning;

  public V3_HorseShooter(
      MotorIOFactory flywheelMotors, MotorIOFactory hoodMotors, BooleanSupplier isHopperRunning) {

    Setpoint<AngularVelocityUnit> flywheelVelocitySetpoint =
        new Setpoint<AngularVelocityUnit>(
            RadiansPerSecond.of(0.0), V3_HorseShooterConstants.SHOOT_CONSTANTS.velocityOffsetStep);
    Setpoint<VoltageUnit> flywheelVoltageSetpoint =
        new Setpoint<VoltageUnit>(
            Volts.of(0.0), V3_HorseShooterConstants.SHOOT_CONSTANTS.voltageOffsetStep);

    Setpoint<AngleUnit> hoodAngleSetpoint =
        new Setpoint<AngleUnit>(
            Radians.of(0.0), V3_HorseShooterConstants.HOOD_CONSTANTS.positionOffsetStep);
    Setpoint<VoltageUnit> hoodVoltageSetpoint =
        new Setpoint<VoltageUnit>(
            Volts.of(0.0), V3_HorseShooterConstants.HOOD_CONSTANTS.voltageOffsetStep);

    this.flywheel =
        new GenericFlywheel(
            flywheelMotors,
            this,
            V3_HorseShooterConstants.SHOOT_CONSTANTS,
            "Flywheel",
            flywheelVelocitySetpoint,
            flywheelVoltageSetpoint);
    this.hood =
        new GenericHood(
            hoodMotors,
            this,
            "Hood",
            V3_HorseShooterConstants.HOOD_CONSTANTS,
            V3_HorseShooterConstants.HOOD_HOMING,
            hoodAngleSetpoint,
            hoodVoltageSetpoint);
    this.shooterGoal = ShooterGoal.STOW;
    this.fixedShotParameters = FixedShots.FRONT_TOWER.getParameters();

    flywheelShootingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        flywheel
                            .getVelocity()
                            .minus(flywheel.getVelocityGoal().getNewSetpoint())
                            .in(RadiansPerSecond))
                    <= V3_HorseShooterConstants.SHOOT_CONSTANTS
                        .constraints
                        .goalTolerance()
                        .get()
                        .in(RadiansPerSecond));
    flywheelFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        flywheel
                            .getVelocity()
                            .minus(flywheel.getVelocityGoal().getNewSetpoint())
                            .in(RadiansPerSecond))
                    <= (V3_HorseShooterConstants.SHOOT_CONSTANTS
                            .constraints
                            .goalTolerance()
                            .get()
                            .in(RadiansPerSecond)
                        + 35));

    hoodFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        hood.getPosition()
                            .minus(hood.getPositionGoal().getNewSetpoint())
                            .in(Radians))
                    <= (V3_HorseShooterConstants.HOOD_CONSTANTS
                            .constraints
                            .goalTolerance()
                            .get()
                            .in(Radians)
                        + Units.degreesToRadians(5)));

    this.isHopperRunning = isHopperRunning;

    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  @Trace
  public void periodic() {
    // A characterization drives the hood or flywheel itself
    if (!characterizing) {
      applyGoal();
    }

    hood.periodic();
    flywheel.periodic();
    appliedGoal = shooterGoal;

    Logger.recordOutput("Shooter/Goal", shooterGoal);

    Logger.recordOutput(
        "Shooter/Hood/Goal Degrees",
        String.format("%.1f", hood.getPositionGoal().getSetpoint().in(Degrees)));
    Logger.recordOutput(
        "Shooter/Hood/Offset Degrees",
        String.format("%.1f", hood.getPositionGoal().getOffset().in(Degrees)));
    Logger.recordOutput(
        "Shooter/Flywheel/Velocity Offset",
        flywheel.getVelocityGoal().getOffset().in(RadiansPerSecond));
    Logger.recordOutput(
        "Shooter/Flywheel/Velocity Magnitude",
        (int) Math.abs(flywheel.getVelocityGoal().getSetpoint().in(RadiansPerSecond)));

    Logger.recordOutput("Shooter/Flywheel At Goal", atGoal());

    Logger.recordOutput(
        "Shooter/Hood At Goal",
        (V3_HorseRobotState.isInAllianceZone())
            ? hood.atPositionGoal()
            : hoodFeedingTrigger.getAsBoolean());

    if (MatchState.getMatchTime() >= 134) {
      shooterGoal = ShooterGoal.STOW;
    }
  }

  private Feedforward getCurrentFeedforward() {
    return Feedforward.amps(
        isHopperRunning.getAsBoolean()
            ? (flywheel
                    .getVelocity()
                    .lt(flywheel.getVelocityGoal().getSetpoint().minus(RadiansPerSecond.of(5)))
                ? V3_HorseShooterConstants.SHOOTER_FEEDFORWARD.get()
                : Amps.zero())
            : Amps.zero());
  }

  private AngularVelocity getIdleVelocity() {
    return V3_HorseRobotState.isInAllianceZone()
        ? V3_HorseRobotState.getScoreVelocity().times(0.9)
        : V3_HorseRobotState.FixedShots.FRONT_TOWER.getParameters().flywheelSpeed().get();
  }

  public Command setGoal(ShooterGoal shooterGoal) {
    return this.run(coroutine -> this.shooterGoal = shooterGoal)
        .named("Shooter: Set Goal To: " + shooterGoal.toString());
  }

  public Command setGoal(Supplier<ShooterGoal> goalSupplier) {
    return this.runRepeatedly(() -> this.shooterGoal = goalSupplier.get())
        .named("Shooter: Follow Goal Supplier");
  }

  public Command runFixedShot(FixedShots shot) {
    return this.run(
            coroutine -> {
              fixedShotParameters = shot.getParameters();
              shooterGoal = ShooterGoal.FIXED_SHOTS;
            })
        .named("Shooter: Run " + shot.toString() + " Fixed Shot");
  }

  public boolean atGoal() {
    return shooterGoal == appliedGoal
        && (V3_HorseRobotState.isInAllianceZone()
            ? (flywheelShootingTrigger.getAsBoolean() && hood.atPositionGoal())
            : (flywheelFeedingTrigger.getAsBoolean() && hoodFeedingTrigger.getAsBoolean()));
  }

  public Command waitUntilAtGoal() {
    return Command.waitUntil(this::atGoal).named("Shooter: Wait Until At Goal");
  }

  public Command waitUntilHoodAtGoal() {
    return hood.waitUntilAtPositionGoal();
  }

  public Command waitUntilFlywheelAtGoal() {
    return flywheel.waitUntilAtVelocityGoal();
  }

  /** Characterizes the hood in voltage and prints its plant and gains. */
  public Command characterizeHood() {
    return characterizing(hood.characterize(), "Characterize Hood");
  }

  /** Characterizes the flywheel in torque current and prints its plant and gains. */
  public Command characterizeFlywheel() {
    return characterizing(flywheel.characterizeTorque(), "Characterize Flywheel");
  }

  /** Characterizes the flywheel in voltage and prints its plant and gains. */
  public Command characterizeFlywheelVoltage() {
    return characterizing(flywheel.characterizeVoltage(), "Characterize Flywheel Voltage");
  }

  /**
   * Runs a command that drives the hood or flywheel itself, such as a characterization or homing,
   * while periodic leaves the hood and flywheel goals alone.
   */
  private Command characterizing(Command characterization, String name) {
    return Command.noRequirements(
            coroutine -> {
              characterizing = true;
              coroutine.await(characterization);
              characterizing = false;
            })
        .whenCanceled(() -> characterizing = false)
        .named("Shooter: " + name);
  }

  private void applyGoal() {
    switch (shooterGoal) {
      case STOW:
        hood.setPositionGoal(V3_HorseShooterConstants.HOOD_CONSTANTS.armParameters.minAngle());
        flywheel.setVelocityGoal(getIdleVelocity(), getCurrentFeedforward());
        break;
      case SCORE:
        hood.setPositionGoal(V3_HorseRobotState.getScoreAngle());
        flywheel.setVelocityGoal(V3_HorseRobotState.getScoreVelocity(), getCurrentFeedforward());
        break;
      case FEED:
        hood.setPositionGoal(V3_HorseRobotState.getFeedAngle());
        flywheel.setVelocityGoal(V3_HorseRobotState.getFeedVelocity(), getCurrentFeedforward());
        break;
      case FIXED_SHOTS:
        hood.setPositionGoal(fixedShotParameters.hoodAngle().get());
        flywheel.setVelocityGoal(
            fixedShotParameters.flywheelSpeed().get(), getCurrentFeedforward());
        break;
      case ZERO:
        hood.setPositionGoal(Radians.of(0.0));
        flywheel.stop();
        break;
      case STOP:
        hood.setPositionGoal(Radians.of(0.0));
        flywheel.setVelocityGoal(getIdleVelocity(), getCurrentFeedforward());
        break;
      default:
        break;
    }
  }

  public Command incrementFlywheelVelocity() {
    return Command.noRequirements(coroutine -> flywheel.getVelocityGoal().increment())
        .named("Shooter: Increment Flywheel Velocity");
  }

  public Command decrementFlywheelVelocity() {
    return Command.noRequirements(coroutine -> flywheel.getVelocityGoal().decrement())
        .named("Shooter: Decrement Flywheel Velocity");
  }

  public Command incrementHoodAngle() {
    return Command.noRequirements(coroutine -> hood.getPositionGoal().increment())
        .named("Shooter: Increment Hood Angle");
  }

  public Command decrementHoodAngle() {
    return Command.noRequirements(coroutine -> hood.getPositionGoal().decrement())
        .named("Shooter: Decrement Hood Angle");
  }

  public Command setHoodAngle(Angle angle) {
    return this.run(
            coroutine -> {
              shooterGoal = ShooterGoal.IDLE;
              hood.setPositionGoal(angle);
            })
        .named("Shooter: Set Hood Angle To " + angle.toString());
  }

  public void setHoodGains(Gains<?, AngleUnit, AngularVelocityUnit> hoodGains) {
    hood.updateGains(SlotConfig.position(hoodGains), GainSlot.ZERO);
  }

  public void setHoodConstraints(AngularPositionConstraints constraints) {
    hood.updateConstraints(constraints);
  }

  public void setFlywheelGains(
      Gains<VoltageUnit, AngularVelocityUnit, AngularVelocityUnit> flywheelGains) {
    flywheel.updateVoltageGains(flywheelGains);
  }

  public void setFlywheelConstraints(AngularVelocityConstraints constraints) {
    flywheel.updateConstraints(constraints);
  }

  public Command setFlywheelVelocity(AngularVelocity velocity) {
    return this.run(
            coroutine -> {
              shooterGoal = ShooterGoal.IDLE;
              flywheel.setVelocityGoal(velocity, Feedforward.amps(Amps.of(0)));
            })
        .named("Shooter: Set Flywheel Velocity To " + velocity.toString());
  }

  /**
   * Homes the hood against its hard stop. Homing is the hood's active control, so periodic stops
   * setting hood goals until it finishes.
   */
  public Command resetHoodZero() {
    return characterizing(hood.home(), "Reset Hood Zero");
  }

  public Angle getHoodAngle() {
    return Radians.of(hood.getPosition().in(Radians));
  }

  public AngularVelocity getFlywheelVelocity() {
    return RadiansPerSecond.of(flywheel.getVelocity().in(RadiansPerSecond));
  }
}
