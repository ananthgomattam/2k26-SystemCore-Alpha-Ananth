package edu.wpi.team190.gompeilib.robot.common.homing;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;

public class HomeableTest extends GompeiLibTestBase {
  @Test
  public void testHomeRunsTheHomingCommand() {
    TestMotors motors = new TestMotors();
    TestMechanism mechanism = new TestMechanism(motors);

    Command home = mechanism.home();
    start(home);
    mechanism.periodic();
    runScheduler();
    runScheduler();
    verify(motors.motor, atLeastOnce())
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(-1), true, CurrentHoming.OPTIONS));
    assertTrue(isRunning(home));
  }
}
