// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc4388.robot.Constants.*;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.LEDPatterns;
import frc4388.utility.controller.IHandController;
import frc4388.utility.controller.XboxController;

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
  private final XboxController m_driverXbox = new XboxController(OIConstants.XBOX_DRIVER_ID);
  private final XboxController m_operatorXbox = new XboxController(OIConstants.XBOX_OPERATOR_ID);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    /* Default Commands */
    // drives the swerve drive with a two-axis input from the driver controller
    m_robotSwerveDrive.setDefaultCommand(
        new RunCommand(() -> m_robotSwerveDrive.driveWithInput(
          XboxController.ClampJoystickAxis(
              getDriverController().getLeftXAxis(),
              getDriverController().getLeftYAxis()), 
          -getDriverController().getRightXAxis(), 
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
    new JoystickButton(getDriverJoystick(), XboxController.Y_BUTTON)
    // new XboxControllerRawButton(m_driverXbox, XboxControllerRaw.Y_BUTTON)
      .whenPressed(() -> m_robotSwerveDrive.m_gyro.reset());

    new JoystickButton(getDriverJoystick(), XboxController.LEFT_BUMPER_BUTTON)
    // new XboxControllerRawButton(m_driverXbox, XboxControllerRaw.LEFT_BUMPER_BUTTON)
      .whenPressed(() -> m_robotSwerveDrive.highSpeed(false));


    new JoystickButton(getDriverJoystick(), XboxController.RIGHT_BUMPER_BUTTON)
    // new XboxControllerRawButton(m_driverXbox, XboxControllerRaw.RIGHT_BUMPER_BUTTON)
      .whenPressed(() -> m_robotSwerveDrive.highSpeed(true));
    
    new JoystickButton(getDriverJoystick(), XboxController.A_BUTTON)

      .whenPressed(() -> resetOdometry());

    /* Operator Buttons */
    // activates "Lit Mode"
    new JoystickButton(getOperatorJoystick(), XboxController.A_BUTTON)
    // new XboxControllerRawButton(m_driverXbox, XboxControllerRaw.A_BUTTON)
        .whenPressed(() -> m_robotLED.setPattern(LEDPatterns.LAVA_RAINBOW))
        .whenReleased(() -> m_robotLED.setPattern(LEDConstants.DEFAULT_PATTERN));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // no auto
    return new InstantCommand();
  }

  /**
   * Add your docs here.
   */
  public IHandController getDriverController() {
    return m_driverXbox;
  }

  public Pose2d getOdometry() {
    return m_robotSwerveDrive.getOdometry();
  }

  public void resetOdometry() {
    m_robotSwerveDrive.resetOdometry();
  }
  /**
   * Add your docs here.
   */
  public IHandController getOperatorController() {
    return m_operatorXbox;
  }

  /**
   * Add your docs here.
   */
  public Joystick getOperatorJoystick() {
    return m_operatorXbox.getJoyStick();
  }

  /**
   * Add your docs here.
   */
  public Joystick getDriverJoystick() {
    return m_driverXbox.getJoyStick();
  }
}
