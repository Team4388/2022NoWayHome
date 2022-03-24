// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.diffplug.common.base.Errors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc4388.utility.RobotTime;
import frc4388.utility.VelocityCorrection;
import frc4388.utility.desmos.DesmosServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final Logger LOGGER = Logger.getLogger(Robot.class.getSimpleName());
  Command m_autonomousCommand;

  private RobotTime m_robotTime = RobotTime.getInstance();
  private RobotContainer m_robotContainer;

  // private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  // private double current;

  // private static DesmosServer desmosServer = new DesmosServer(8000);

  public static Alliance alliance;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    LOGGER.log(Level.ALL, "Logging Test 1/8");
    LOGGER.log(Level.SEVERE, "Logging Test 2/8");
    LOGGER.log(Level.WARNING, "Logging Test 3/8");
    LOGGER.log(Level.INFO, "Logging Test 4/8");
    LOGGER.log(Level.CONFIG, "Logging Test 5/8");
    LOGGER.log(Level.FINE, "Logging Test 6/8");
    LOGGER.log(Level.FINER, "Logging Test 7/8");
    LOGGER.log(Level.FINEST, "Logging Test 8/8");

    // var path =
    // PathPlannerUtil.Path.read(Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("Move
    // Forward.path").toFile());
    // LOGGER.finest(path::toString);
    LOGGER.fine("robotInit()");



    // LOGGER.fine("Sssssssssh.");
    // DriverStation.silenceJoystickConnectionWarning(true);
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // addPeriodic(m_robotContainer::recordPeriodic, kDefaultPeriod);
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("JVM Memory", new RunCommand(() -> {
    }) {
      @Override
      public boolean runsWhenDisabled() {
        return true;
      }

      @Override
      public String getName() {
        if (isScheduled()) {
          Runtime runtime = Runtime.getRuntime();
          long totalMemory = runtime.totalMemory() / 1_000_000;
          long freeMemory = runtime.freeMemory() / 1_000_000;
          long maxMemory = runtime.maxMemory() / 1_000_000;
          return totalMemory - freeMemory + " MB / " + totalMemory + " MB / " + maxMemory + " MB";
        }
        return "Not Running";
      }
    });
    SmartDashboard.putData("Usable Deploy Space", new RunCommand(() -> {
    }) {
      @Override
      public boolean runsWhenDisabled() {
        return true;
      }

      @Override
      public String getName() {
        if (isScheduled()) {
          File deploy = Filesystem.getDeployDirectory();
          long usedSpace = Errors.suppress().getWithDefault(
              () -> Files.walk(deploy.toPath()).map(Path::toFile).filter(File::isFile).mapToLong(File::length).sum(),
              0l) / 1_000_000;
          long usableSpace = deploy.getUsableSpace() / 1_000_000;
          return usedSpace + " MB / " + usableSpace + " MB";
        }
        return "Not Running";
      }
    });

    // desmosServer.start();
    m_robotContainer.m_robotVisionOdometry.setLEDs(true);
    // DesmosServer.putTable("table", "x1", new double[] {44}, "y1", new double[] {0});
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_robotTime.updateTimes();
    System.out.println((180.0 / Math.PI) * Math.atan2(-(219.25 / 2.00) - 40.44 + 10.00, -(82.83 / 2.00) + 15.56));
    // current = 
      // m_robotContainer.m_robotBoomBoom.getCurrent() +
      // m_robotContainer.m_robotClimber.getCurrent(); //+
      // m_robotContainer.m_robotHood.getCurrent() +
      // m_robotContainer.m_robotIntake.getCurrent() +
      // m_robotContainer.m_robotExtender.getCurrent() +
      // m_robotContainer.m_robotSerializer.getCurrent() +
      // m_robotContainer.m_robotStorage.getCurrent() +
      // m_robotContainer.m_robotSwerveDrive.getCurrent();
      // m_robotContainer.m_robotTurret.getCurrent();
    // SmartDashboard.putNumber("Total Robot Current Draw", current);
    // SmartDashboard.putNumber("Drive Train Voltage", m_robotContainer.m_robotSwerveDrive.getVoltage());
    // SmartDashboard.putNumber("Drive Train Current", m_robotContainer.m_robotSwerveDrive.getCurrent());
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // print odometry data to smart dashboard for debugging (if causing timeout
    // errors, you can comment it)
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    LOGGER.fine("disabledInit()");
    m_robotTime.endMatchTime();
    if (isTest()) {
      // IMPORTANT: Had to chown the pathplanner folder in order to save autos.
      File outputFile = Filesystem.getDeployDirectory().toPath().resolve("pathplanner")
          .resolve("recording." + System.currentTimeMillis() + ".path").toFile();
      if (Boolean.TRUE.equals(Errors.log().getWithDefault(outputFile::createNewFile, false))) {
        // m_robotContainer.createPath(null, null, false).write(outputFile);
        LOGGER.log(Level.SEVERE, "Recorded path to {0}.", outputFile.getPath());
      } else
        LOGGER.log(Level.SEVERE, "Unable to record path to {0}", outputFile.getPath());
    }

    m_robotContainer.m_robotVisionOdometry.setLEDs(true);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    LOGGER.fine("autonomousInit()");

    Robot.alliance = DriverStation.getAlliance();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotTime.startMatchTime();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    LOGGER.fine("teleopInit()");

    Robot.alliance = DriverStation.getAlliance();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotTime.startMatchTime();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
