// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;


import java.lang.reflect.Array;
import java.nio.file.FileSystem;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import javax.swing.plaf.nimbus.State;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc4388.robot.Constants.*;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.LEDPatterns;
import frc4388.utility.controller.DeadbandedXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* RobotMap */
  private final RobotMap m_robotMap = new RobotMap();

  /* Subsystems */
  private final SwerveDrive m_robotSwerveDrive = new SwerveDrive(
    m_robotMap.leftFront, m_robotMap.leftBack, m_robotMap.rightFront, m_robotMap.rightBack, m_robotMap.gyro);

  private final LED m_robotLED = new LED(m_robotMap.LEDController);

  /* Controllers */
  private final XboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
  private final XboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    /* Default Commands */
    // drives the swerve drive with a two-axis input from the driver controller
    m_robotSwerveDrive.setDefaultCommand(
        new RunCommand(() -> m_robotSwerveDrive.driveWithInput(
              getDriverController().getLeftX(),
              getDriverController().getLeftY(),
              getDriverController().getRightX(),
              getDriverController().getRightY(),
          true),
           m_robotSwerveDrive));

    // continually sends updates to the Blinkin LED controller to keep the lights on
    m_robotLED.setDefaultCommand(new RunCommand(m_robotLED::updateLED, m_robotLED));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    new JoystickButton(getDriverController(), XboxController.Button.kY.value)
    // new XboxControllerRawButton(m_driverXbox, XboxControllerRaw.Y_BUTTON)
      .whenPressed(m_robotSwerveDrive.m_gyro::reset);

    new JoystickButton(getDriverController(), XboxController.Button.kLeftBumper.value)
    // new XboxControllerRawButton(m_driverXbox, XboxControllerRaw.LEFT_BUMPER_BUTTON)
      .whenPressed(() -> m_robotSwerveDrive.highSpeed(false));


    new JoystickButton(getDriverController(), XboxController.Button.kRightBumper.value)
    // new XboxControllerRawButton(m_driverXbox, XboxControllerRaw.RIGHT_BUMPER_BUTTON)
      .whenPressed(() -> m_robotSwerveDrive.highSpeed(true));
    
    new JoystickButton(getDriverController(), XboxController.Button.kA.value)

      .whenPressed(() -> resetOdometry(new Pose2d(0, 0, new Rotation2d(0))));
      //.whenPressed(this::resetOdometry);

    /* Operator Buttons */
    // activates "Lit Mode"
    new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
    // new XboxControllerRawButton(m_driverXbox, XboxControllerRaw.A_BUTTON)
        .whenPressed(() -> m_robotLED.setPattern(LEDPatterns.LAVA_RAINBOW))
        .whenReleased(() -> m_robotLED.setPattern(LEDConstants.DEFAULT_PATTERN));
  }

  public SequentialCommandGroup runAuto(String path, double maxVel, double maxAccel) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(path, maxVel, maxAccel);

    PIDController xController = SwerveDriveConstants.X_CONTROLLER;
    PIDController yController = SwerveDriveConstants.Y_CONTROLLER;
    ProfiledPIDController thetaController = SwerveDriveConstants.THETA_CONTROLLER;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    //thetaController.reset(new TrapezoidProfile.State(m_robotSwerveDrive.getOdometry()));

    PPSwerveControllerCommand ppSCC = new PPSwerveControllerCommand(
        traj,
        m_robotSwerveDrive::getOdometry,
        m_robotSwerveDrive.m_kinematics,
        xController,
        yController,
        thetaController,
        m_robotSwerveDrive::setModuleStates,
        m_robotSwerveDrive);
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotSwerveDrive.m_gyro.reset()),
      new InstantCommand(() -> m_robotSwerveDrive.resetOdometry(traj.getInitialPose())),
      ppSCC,
      new InstantCommand(() -> m_robotSwerveDrive.stopModules())
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // https://github.com/mjansen4857/pathplanner/wiki <-- Pathplanner Wik

    // PathPlannerTrajectory ppfirstTestPath = PathPlanner.loadPath("First Test Path", 4.0, 4.0);
    // PathPlannerTrajectory ppMoveForward = PathPlanner.loadPath("Move Forward", 1.0, 1.0);
    // PathPlannerTrajectory ppRotate = PathPlanner.loadPath("Rotate", 1.0, 1.0);

    // PathPlannerTrajectory ppCurrent = PathPlanner.loadPath("First Test Path", 1.0, 1.0); // change this to change auto

    // PIDController xController = SwerveDriveConstants.X_CONTROLLER;
    // PIDController yController = SwerveDriveConstants.Y_CONTROLLER;
    // ProfiledPIDController thetaController = SwerveDriveConstants.THETA_CONTROLLER;
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // PPSwerveControllerCommand ppSwerveControllerCommand = new PPSwerveControllerCommand(
    //   ppCurrent,
    //   m_robotSwerveDrive::getOdometry,
    //   m_robotSwerveDrive.m_kinematics,
    //   xController,
    //   yController,
    //   thetaController,
    //   m_robotSwerveDrive::setModuleStates,
    //   m_robotSwerveDrive
    // );

    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> m_robotSwerveDrive.m_gyro.reset()),
    //   new InstantCommand(() -> m_robotSwerveDrive.resetOdometry(ppCurrent.getInitialPose())),
    //   ppSwerveControllerCommand,
    //   new InstantCommand(() -> m_robotSwerveDrive.stopModules())
    // );
    return runAuto("Move Forward", 5.0, 5.0);
  }

  /**
   * Add your docs here.
   */
  public XboxController getDriverController() {
    return m_driverXbox;
  }

  public Pose2d getOdometry() {
    return m_robotSwerveDrive.getOdometry();
  }

  public void resetOdometry(Pose2d pose) {
    m_robotSwerveDrive.resetOdometry(pose);
  }
  /**
   * Add your docs here.
   */
  public XboxController getOperatorController() {
    return m_operatorXbox;
  }
}
