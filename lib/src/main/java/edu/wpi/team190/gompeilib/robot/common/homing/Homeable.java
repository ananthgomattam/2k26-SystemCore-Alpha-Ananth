package edu.wpi.team190.gompeilib.robot.common.homing;

import edu.wpi.team190.gompeilib.robot.common.core.GompeiMechanism;
import org.wpilib.command3.Command;

/** A mechanism that finds its position by driving into a hard stop. */
public interface Homeable extends GompeiMechanism {
  /**
   * Returns the mechanism's homing.
   *
   * @return the homing
   * @throws IllegalStateException if the mechanism wasn't configured to home
   */
  CurrentHoming<?> homing();

  /**
   * Returns a command that homes the mechanism.
   *
   * @return the command
   * @throws IllegalStateException if the mechanism wasn't configured to home
   */
  default Command home() {
    return homing().command();
  }
}
