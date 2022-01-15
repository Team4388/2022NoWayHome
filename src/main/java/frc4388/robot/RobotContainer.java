// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.*;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.LEDPatterns;

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
    m_robotMap.leftFrontSteerMotor, m_robotMap.leftFrontWheelMotor,
    m_robotMap.rightFrontSteerMotor, m_robotMap.rightFrontWheelMotor,
    m_robotMap.leftBackSteerMotor, m_robotMap.leftBackWheelMotor,
    m_robotMap.rightBackSteerMotor, m_robotMap.rightBackWheelMotor,
    m_robotMap.leftFrontEncoder,
    m_robotMap.rightFrontEncoder,
    m_robotMap.leftBackEncoder,
    m_robotMap.rightBackEncoder,
    m_robotMap.gyro
  );

  private final LED m_robotLED = new LED(m_robotMap.LEDController);

  /* Controllers */
  private final XboxController m_driverXbox = new XboxController(OIConstants.XBOX_DRIVER_ID);
  private final XboxController m_operatorXbox = new XboxController(OIConstants.XBOX_OPERATOR_ID);

  private static final boolean BYPASS_DS_CONTROLLER = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    /* Default Commands */
    // drives the swerve drive with a two-axis input from the driver controller
    m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        float[] joystickAxes = new float[HAL.kMaxJoystickAxes];
        if (BYPASS_DS_CONTROLLER) HAL.getJoystickAxes((byte) getDriverController().getPort(), joystickAxes);
        double leftXAxis = BYPASS_DS_CONTROLLER ? joystickAxes[XboxController.Axis.kLeftX.value] : getDriverController().getLeftX();
        double leftYAxis = BYPASS_DS_CONTROLLER ? joystickAxes[XboxController.Axis.kLeftY.value] : getDriverController().getLeftY();
        double rightXAxis = BYPASS_DS_CONTROLLER ?  joystickAxes[XboxController.Axis.kRightX.value] : getDriverController().getRightX();
        m_robotSwerveDrive.driveWithInput(clampJoystickAxis(leftXAxis, leftYAxis), -rightXAxis, true);
      }, m_robotSwerveDrive
    ));

    // continually sends updates to the Blinkin LED controller to keep the lights on
    m_robotLED.setDefaultCommand(new RunCommand(m_robotLED::updateLED, m_robotLED));
  }
  private static double[] clampJoystickAxis(double x, double y) {
    double squareMag = x * x + y * y;
    double[] ret = { x, y };
    if (squareMag > 1.d) {
      double mag = Math.sqrt(squareMag);
      ret[0] = x / mag;
      ret[1] = y / mag;
    }
    SmartDashboard.putNumberArray("Input", new double[] {squareMag, x, y, ret[0], ret[1]});
    return ret;
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    /*new JoystickButton(getDriverJoystick(), XboxController.Y_BUTTON)
      .whenPressed(() -> m_robotSwerveDrive.resetGyro());*/
    /* Operator Buttons */
    // activates "Lit Mode"
    new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
        .whenPressed(() -> m_robotLED.setPattern(LEDPatterns.LAVA_RAINBOW))
        .whenReleased(() -> m_robotLED.setPattern(LEDConstants.DEFAULT_PATTERN));

    new JoystickButton(getDriverController(), XboxController.Button.kA.value)
    .whenPressed(() -> {
      m_robotSwerveDrive.m_gyro.setYaw(0);
    });
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
  public XboxController getDriverController() {
    return m_driverXbox;
  }

  /**
   * Add your docs here.
   */
  public XboxController getOperatorController() {
    return m_operatorXbox;
  }
}
