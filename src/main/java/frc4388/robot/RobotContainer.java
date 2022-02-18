// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import java.util.ArrayList;
import java.util.Objects;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(
    m_robotMap.leftFront, m_robotMap.leftBack, m_robotMap.rightFront, m_robotMap.rightBack, m_robotMap.gyro);

  private final TalonFX m_testMotor = new TalonFX(23);

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

    m_testMotor.set(TalonFXControlMode.PercentOutput, 0);
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
      .whenPressed(() -> m_robotSwerveDrive.resetGyro());

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

  /**
   * Generate autonomous
   * @param maxVel max velocity for the path (null to override default value of 5.0)
   * @param maxAccel max acceleration for the path (null to override default value of 5.0)
   * @param inputs strings (path names) or commands you want to run (in order)
   * @return array of commands, which can then be processed in a command group
   */
  public Command[] buildAuto(Double maxVel, Double maxAccel, Object... inputs) {

    // default vel and acc
    maxVel = Objects.requireNonNullElse(maxVel, SwerveDriveConstants.MAX_VEL);
    maxAccel = Objects.requireNonNullElse(maxAccel, SwerveDriveConstants.MAX_ACC);

    ArrayList<Command> commands = new ArrayList<Command>();
    commands.add(new InstantCommand(() -> m_robotSwerveDrive.m_gyro.reset()));

    // pids controlling the path
    PIDController xController = SwerveDriveConstants.X_CONTROLLER;
    PIDController yController = SwerveDriveConstants.Y_CONTROLLER;
    ProfiledPIDController thetaController = SwerveDriveConstants.THETA_CONTROLLER;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // parse input
    for (int i=0; i<inputs.length; i++) {

      // if string, process as pathplanner trajectory
      if (inputs[i] instanceof String) {

        PathPlannerTrajectory traj = PathPlanner.loadPath(inputs[i].toString(), maxVel, maxAccel);
        PathPlannerState initState = (PathPlannerState) traj.sample(0);

        commands.add(new InstantCommand(() -> m_robotSwerveDrive.resetOdometry(new Pose2d(initState.poseMeters.getTranslation(), initState.holonomicRotation))));
        commands.add(new PPSwerveControllerCommand(
                          traj,
                          m_robotSwerveDrive::getOdometry,
                          m_robotSwerveDrive.m_kinematics,
                          xController,
                          yController,
                          thetaController,
                          m_robotSwerveDrive::setModuleStates,
                          m_robotSwerveDrive));
      }

      // if command, just add it to the array
      if (inputs[i] instanceof Command) {
        commands.add((Command) inputs[i]);
      }
    }
    
    commands.add(new InstantCommand(() -> m_robotSwerveDrive.stopModules()));
    Command[] ret = new Command[commands.size()];
    ret = commands.toArray(ret);
    return ret;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // https://github.com/mjansen4857/pathplanner/wiki <-- Pathplanner Wiki
    return new ParallelCommandGroup(
            buildAuto(
              null,
              null,
              new SequentialCommandGroup(buildAuto(0.5, 0.5, "Move Forward", "Move Down")), 
              new InstantCommand(() -> m_testMotor.set(TalonFXControlMode.PercentOutput, 0.2))
            )
          );
  }

  /**
   * Add your docs here.
   */
  public XboxController getDriverController() {
    return m_driverXbox;
  }

  /**
   * Get odometry.
   * @return Odometry
   */
  public Pose2d getOdometry() {
    return m_robotSwerveDrive.getOdometry();
  }

  /**
   * Set odometry to given pose.
   * @param pose Pose to set odometry to.
   */
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
