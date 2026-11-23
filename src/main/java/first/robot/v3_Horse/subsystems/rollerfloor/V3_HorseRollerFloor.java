package first.robot.v3_Horse.subsystems.rollerfloor;

import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.robot.mechanisms.roller.GenericRoller;
import first.robot.v3_Horse.V3_HorseRobotState;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloorConstants.RollerFloorState;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.MatchState;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Voltage;

public class V3_HorseRollerFloor implements Mechanism {
  private final GenericRoller rollerFloor;
  @Getter private RollerFloorState rollerFloorGoal;
  private final Setpoint<VoltageUnit> rollerFloorSetpoint,
      rollerFloorOverrideSetpoint,
      rollerFloorSpinUpSetpoint;

  private final Setpoint<VoltageUnit> rollerFloorStopSetpoint;

  public V3_HorseRollerFloor(MotorIOFactory rollerFloorMotors) {
    rollerFloorGoal = RollerFloorState.STOP;
    rollerFloorSetpoint =
        new Setpoint<>(
            V3_HorseRollerFloorConstants.ROLLER_FLOOR_RUN_VOLTAGE,
            V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));
    rollerFloorOverrideSetpoint =
        new Setpoint<>(
            V3_HorseRollerFloorConstants.ROLLER_FLOOR_OVERRIDE_VOLTAGE,
            V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));
    rollerFloorSpinUpSetpoint =
        new Setpoint<>(
            Volts.of(-1.5),
            V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep,
            Volts.of(0),
            Volts.of(0));

    rollerFloorStopSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep,
            Volts.of(0),
            Volts.of(0));

    rollerFloor =
        new GenericRoller(
            rollerFloorMotors,
            this,
            V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS,
            "Roller",
            rollerFloorSetpoint);

    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public void periodic() {
    switch (rollerFloorGoal) {
      case RUN:
        V3_HorseRobotState.setHopperRunning(true);
        runRollerFloor(rollerFloorSetpoint);
        break;
      case OVERRIDE:
        V3_HorseRobotState.setHopperRunning(false);
        runRollerFloor(rollerFloorOverrideSetpoint);
        break;
      case STOP:
        V3_HorseRobotState.setHopperRunning(false);
        stopRollerFloor();
        break;
      case SPIN_UP:
        V3_HorseRobotState.setHopperRunning(false);
        runRollerFloor(rollerFloorSpinUpSetpoint);
        break;
      default:
        V3_HorseRobotState.setHopperRunning(false);
        runRollerFloor(rollerFloorSetpoint);
        break;
    }
    rollerFloor.periodic();
    Logger.recordOutput(
        "Elastic/Hopper/RollerFloor/Voltage Magnitude",
        String.format("%.1f", Math.abs(rollerFloor.getVoltageGoal().getSetpoint().in(Volts))));
    Logger.recordOutput(
        "Elastic/Hopper/RollerFloor/Voltage Offset",
        String.format("%.1f", rollerFloor.getVoltageGoal().getOffset().in(Volts)));

    if (MatchState.getMatchTime() >= 134) {
      rollerFloorGoal = RollerFloorState.STOP;
    }
  }

  public Command setState(RollerFloorState state) {
    return Command.noRequirements(
            coroutine -> {
              rollerFloorGoal = state;
            })
        .named("RollerFloor State Set To " + state.toString());
  }

  public void setRollerFloorVoltage(Voltage voltage) {
    rollerFloorSetpoint.setSetpoint(voltage);
  }

  public void setOverrideRollerFloorVoltage(Voltage voltage) {
    rollerFloorOverrideSetpoint.setSetpoint(voltage);
  }

  public void runRollerFloor(Setpoint<VoltageUnit> voltageSetpoint) {
    rollerFloor.setVoltageGoal(voltageSetpoint);
  }

  public void stopRollerFloor() {
    rollerFloor.setVoltageGoal(rollerFloorStopSetpoint);
  }

  public Command incrementRollerFloorVelocity() {
    return Command.noRequirements(
            coroutine -> {
              if (rollerFloorGoal.equals(RollerFloorState.RUN)) {
                rollerFloorSetpoint.increment();
              } else if (rollerFloorGoal.equals(RollerFloorState.OVERRIDE)) {
                rollerFloorOverrideSetpoint.increment();
              } else {
                stopRollerFloor();
              }
            })
        .named("Roller Floor: Increment Voltage");
  }

  public Command decrementRollerFloorVelocity() {
    return Command.noRequirements(
            coroutine -> {
              if (rollerFloorGoal.equals(RollerFloorState.RUN)) {
                rollerFloorSetpoint.decrement();
              } else if (rollerFloorGoal.equals(RollerFloorState.OVERRIDE)) {
                rollerFloorOverrideSetpoint.decrement();
              } else {
                stopRollerFloor();
              }
            })
        .named("Roller Floor: Decrement Voltage");
  }
}
