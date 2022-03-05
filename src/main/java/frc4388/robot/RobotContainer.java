// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.*;
import frc4388.robot.subsystems.Claws;
import frc4388.robot.commands.RunClaw;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Claws.ClawType;
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

  /* Subsystems *//*
  private final SwerveDrive m_robotSwerveDrive = new SwerveDrive(
    m_robotMap.leftFrontSteerMotor, m_robotMap.leftFrontWheelMotor,
    m_robotMap.rightFrontSteerMotor, m_robotMap.rightFrontWheelMotor,
    m_robotMap.leftBackSteerMotor, m_robotMap.leftBackWheelMotor,
    m_robotMap.rightBackSteerMotor, m_robotMap.rightBackWheelMotor,
    m_robotMap.leftFrontEncoder,
    m_robotMap.rightFrontEncoder,
    m_robotMap.leftBackEncoder,
    m_robotMap.rightBackEncoder
  );*/

  /*private final LED m_robotLED = new LED(m_robotMap.LEDController);

  private final Climber m_robotClimber = new Climber(m_robotMap.shoulder, m_robotMap.elbow, m_robotMap.gyro, false);
  */
  private final Claws m_claws = new Claws(m_robotMap.leftClaw, m_robotMap.rightClaw);

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
    // m_robotSwerveDrive.setDefaultCommand(
    //     new RunCommand(() -> m_robotSwerveDrive.driveWithInput(-getDriverController().getLeftXAxis(),
    //         getDriverController().getLeftYAxis(), -getDriverController().getRightXAxis(), false), m_robotSwerveDrive));

    // moves climber in xy space with two-axis input from the operator controller
    /*m_robotClimber.setDefaultCommand(
        new RunCommand(() -> m_robotClimber.controlWithInput(getOperatorController().getLeftXAxis(),
            getOperatorController().getLeftYAxis()), m_robotClimber));

    // continually sends updates to the Blinkin LED controller to keep the lights on
    m_robotLED.setDefaultCommand(new RunCommand(m_robotLED::updateLED, m_robotLED));*/
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */

    /* Operator Buttons */
    // activates "Lit Mode"
    /*new JoystickButton(getOperatorJoystick(), XboxController.A_BUTTON)
        .whenPressed(() -> m_robotLED.setPattern(LEDPatterns.LAVA_RAINBOW))
        .whenReleased(() -> m_robotLED.setPattern(LEDConstants.DEFAULT_PATTERN));*/

    // run claws
    new JoystickButton(getOperatorJoystick(), XboxController.Y_BUTTON)
      .whenPressed(new RunClaw(m_claws, ClawType.LEFT, true))
      .whenPressed(new RunClaw(m_claws, ClawType.RIGHT, true));
    
    new JoystickButton(getOperatorJoystick(), XboxController.X_BUTTON)
      .whenPressed(new RunClaw(m_claws, ClawType.LEFT, false))
      .whenPressed(new RunClaw(m_claws, ClawType.RIGHT, false));

    // new JoystickButton(getOperatorJoystick(), XboxController.Y_BUTTON)
    //     .whenPressed(() -> m_claws.setSpeed(0.5))
    //     .whenReleased(() -> m_claws.setSpeed(0.0));
    // new JoystickButton(getOperatorJoystick(), XboxController.X_BUTTON)
    //     .whenPressed(() -> m_claws.setSpeed(-0.5))
    //     .whenReleased(() -> m_claws.setSpeed(0.0));
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
