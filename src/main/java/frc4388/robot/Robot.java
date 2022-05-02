package frc4388.robot;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final Logger LOGGER = Logger.getLogger(Robot.class.getSimpleName());
  Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // private static DesmosServer desmosServer = new DesmosServer(8000);

  /**
   * This function is run when the robot is first started up and should be used for any initialization
   * code.
   */
  @Override
  public void robotInit() {
    LOGGER.fine("robotInit()");
    LOGGER.log(Level.ALL, "Logging Test 1/8");
    LOGGER.log(Level.SEVERE, "Logging Test 2/8");
    LOGGER.log(Level.WARNING, "Logging Test 3/8");
    LOGGER.log(Level.INFO, "Logging Test 4/8");
    LOGGER.log(Level.CONFIG, "Logging Test 5/8");
    LOGGER.log(Level.FINE, "Logging Test 6/8");
    LOGGER.log(Level.FINER, "Logging Test 7/8");
    LOGGER.log(Level.FINEST, "Logging Test 8/8");

    // Instantiate our RobotContainer. This will perform all our button bindings, and put our autonomous
    // chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // desmosServer.start();

    m_robotContainer.m_robotVisionOdometry.setLEDs(false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running
    // subsystem periodic() methods. This must be called from the robot's periodic block in order for
    // anything in the Command-based framework to work.
    m_robotContainer.updateValues();
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You can use it to reset
   * any subsystem information you want to clear when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    LOGGER.fine("disabledInit()");

    m_robotContainer.m_robotVisionOdometry.setLEDs(false);
  }

  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    LOGGER.fine("autonomousInit()");

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    LOGGER.fine("teleopInit()");

    DriverStation.silenceJoystickConnectionWarning(true);

    // This makes sure that the autonomous stops running when teleop starts running.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}
}
