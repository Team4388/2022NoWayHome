// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.Constants.SwerveDriveConstants;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {

  public RobotMap() {
    configureLEDMotorControllers();
    //configureSwerveMotorControllers();
  }

  /* LED Subsystem */
  public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);

  void configureLEDMotorControllers() {
    
  }
  
  /* Swerve Subsystem */
  // public final WPI_TalonFX leftFrontSteerMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_FRONT_STEER_CAN_ID);
  // public final WPI_TalonFX leftFrontWheelMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_FRONT_WHEEL_CAN_ID);
  // public final WPI_TalonFX rightFrontSteerMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_FRONT_STEER_CAN_ID);
  // public final WPI_TalonFX rightFrontWheelMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_FRONT_WHEEL_CAN_ID);
  // public final WPI_TalonFX leftBackSteerMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_BACK_STEER_CAN_ID);
  // public final WPI_TalonFX leftBackWheelMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_BACK_WHEEL_CAN_ID);
  // public final WPI_TalonFX rightBackSteerMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_BACK_STEER_CAN_ID);
  // public final WPI_TalonFX rightBackWheelMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_BACK_WHEEL_CAN_ID);
  // public final CANCoder leftFrontEncoder = new CANCoder(SwerveDriveConstants.LEFT_FRONT_STEER_CAN_ENCODER_ID);
  // public final CANCoder rightFrontEncoder = new CANCoder(SwerveDriveConstants.RIGHT_FRONT_STEER_CAN_ENCODER_ID);
  // public final CANCoder leftBackEncoder = new CANCoder(SwerveDriveConstants.LEFT_BACK_STEER_CAN_ENCODER_ID);
  // public final CANCoder rightBackEncoder = new CANCoder(SwerveDriveConstants.RIGHT_BACK_STEER_CAN_ENCODER_ID);
      
  // void configureSwerveMotorControllers() {
  //   leftFrontEncoder.configMagnetOffset(SwerveDriveConstants.LEFT_BACK_ENCODER_OFFSET);
  //   rightFrontEncoder.configMagnetOffset(SwerveDriveConstants.RIGHT_FRONT_ENCODER_OFFSET);
  //   leftBackEncoder.configMagnetOffset(SwerveDriveConstants.LEFT_BACK_ENCODER_OFFSET);
  //   rightBackEncoder.configMagnetOffset(SwerveDriveConstants.RIGHT_BACK_ENCODER_OFFSET);

  //   leftFrontSteerMotor.configFactoryDefault();
  //   leftFrontWheelMotor.configFactoryDefault();
  //   rightFrontSteerMotor.configFactoryDefault();
  //   rightFrontWheelMotor.configFactoryDefault();
  //   leftBackSteerMotor.configFactoryDefault();
  //   leftBackWheelMotor.configFactoryDefault();
  //   rightBackSteerMotor.configFactoryDefault();
  //   rightBackWheelMotor.configFactoryDefault();

  //   leftFrontSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   leftFrontWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightFrontSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightFrontWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   leftBackSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   leftBackWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightBackSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightBackWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

  //   leftFrontWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   leftFrontSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightFrontSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightFrontWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   leftBackSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   leftBackWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightBackSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightBackWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

  //   leftFrontWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   leftFrontSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightFrontSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightFrontWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   leftBackSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   leftBackWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightBackSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //   rightBackWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

  //   // config cancoder as remote encoder for swerve steer motors
    //leftFrontSteerMotor.configRemoteFeedbackFilter(leftFrontEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    //leftBackSteerMotor.configRemoteFeedbackFilter(leftBackEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    //rightFrontSteerMotor.configRemoteFeedbackFilter(rightFrontEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    //rightBackSteerMotor.configRemoteFeedbackFilter(rightBackEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  //}

  //Shooter Config
  /*Boom Boom Subsystem*/ 
  public final WPI_TalonFX shooterFalconLeft = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_LEFT_CAN_ID);
  public final WPI_TalonFX shooterFalconRight = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_RIGHT_CAN_ID);
  
  // public final CANSparkMax shooterTurret = new CANSparkMax(ShooterConstants.TURRET_MOTOR_CAN_ID, MotorType.kBrushless);
  // Create motor CANSparkMax
  void ConfigureShooterMotorControllers() {

    //LEFT FALCON
    shooterFalconLeft.configFactoryDefault();
    shooterFalconLeft.setNeutralMode(NeutralMode.Coast);
    shooterFalconLeft.setInverted(true);
    shooterFalconLeft.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configClosedloopRamp(0.75, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configPeakOutputReverse(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configClosedLoopPeriod(0, ShooterConstants.CLOSED_LOOP_TIME_MS, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG, ShooterConstants.SHOOTER_TIMEOUT_MS);

    
    
    //RIGHT FALCON
    shooterFalconRight.setInverted(false);
    shooterFalconRight.setNeutralMode(NeutralMode.Coast);
    shooterFalconRight.configFactoryDefault();
    shooterFalconRight.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configClosedloopRamp(0.75, ShooterConstants.SHOOTER_TIMEOUT_MS);
    //m_shooterFalconRight.configPeakOutputForward(0, ShooterConstants.SHOOTER_TIMEOUT_MS);(comment it in if necessary)
    shooterFalconRight.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configClosedLoopPeriod(0,ShooterConstants.CLOSED_LOOP_TIME_MS, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG, ShooterConstants.SHOOTER_TIMEOUT_MS);
  /*Turret Subsytem*/
  
  
  shooterFalconRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,     6,                9,                4.2));
  shooterFalconLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,      12,                13,                0.4));

  }


}
