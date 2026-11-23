package edu.wpi.team190.gompeilib;

import edu.wpi.team190.gompeilib.robot.RobotMode;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;

/**
 * Base fixture for GompeiLib tests.
 *
 * <p>Initializes the WPILib HAL and sets up the GompeiLib environment in SIM mode before each test,
 * and resets/closes registered AutoCloseable resources and deinitializes GompeiLib after each test.
 */
public abstract class GompeiLibTestBase {
  private final List<AutoCloseable> autoCloseables = new ArrayList<>();
  protected Scheduler scheduler;

  @BeforeEach
  public void setUpGompeiLib() {
    assert HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception ignored) {
    }
    GompeiLib.init(RobotMode.SIM, isTuningMode(), loopPeriodSecs());
    scheduler = Scheduler.createIndependentScheduler();
  }

  @AfterEach
  public void tearDownGompeiLib() {
    for (AutoCloseable resource : autoCloseables) {
      if (resource != null) {
        try {
          resource.close();
        } catch (Exception ignored) {
        }
      }
    }
    autoCloseables.clear();
    try {
      GompeiLib.deinit();
    } catch (Exception ignored) {
    }
  }

  /**
   * Registers an AutoCloseable resource to be closed in @AfterEach.
   *
   * @param resource the resource to close
   * @param <T> type of AutoCloseable
   * @return the registered resource
   */
  public <T extends AutoCloseable> T registerCloseable(T resource) {
    if (resource != null) {
      autoCloseables.add(resource);
    }
    return resource;
  }

  /**
   * Override to customize whether tuning mode is enabled during test setup.
   *
   * @return true if tuning mode is enabled, false otherwise (default: false)
   */
  protected void start(Command command) {
    scheduler.schedule(command);
    scheduler.run();
  }

  protected void runScheduler() {
    scheduler.run();
  }

  protected boolean isRunning(Command command) {
    return scheduler.isRunning(command);
  }

  protected boolean isTuningMode() {
    return false;
  }

  /**
   * Override to customize loop period during test setup.
   *
   * @return loop period in seconds (default: 0.005)
   */
  protected double loopPeriodSecs() {
    return 0.005;
  }

  /**
   * Re-initializes GompeiLib with specific settings during a test.
   *
   * @param mode robot mode
   * @param isTuning whether tuning mode is enabled
   * @param loopPeriodSecs loop period in seconds
   */
  protected void reinitGompeiLib(RobotMode mode, boolean isTuning, double loopPeriodSecs) {
    try {
      GompeiLib.deinit();
    } catch (Exception ignored) {
    }
    GompeiLib.init(mode, isTuning, loopPeriodSecs);
  }
}
