package edu.wpi.team190.gompeilib.robot.mechanisms.hood;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.common.homing.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.ArmConstants;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.HomingArm;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;

/** A hood: a short-range {@link HomingArm}; everything else is the arm's. */
public class GenericHood extends HomingArm {
  /**
   * Creates a hood.
   *
   * @param motorFactory how to build the hood's motors
   * @param subsystem the robot mechanism that owns the hood
   * @param name the hood's name; it logs under {@code <subsystem>/<name>}
   * @param constants the hood's constants
   * @param homing how the hood homes
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public GenericHood(
      MotorIOFactory motorFactory,
      Mechanism subsystem,
      String name,
      ArmConstants constants,
      CurrentHoming.Config<AngleUnit> homing,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    super(motorFactory, subsystem, name, constants, homing, positionGoal, voltageGoal);
  }

  @Override
  @Trace
  public void periodic() {
    core().periodic();
  }
}
