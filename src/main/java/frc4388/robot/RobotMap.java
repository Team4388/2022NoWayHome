// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.SwerveModule;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {

  public RobotMap() {
    configureLEDMotorControllers();
    configureSwerveMotorControllers();
  }

  /* LED Subsystem */
  public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);

  void configureLEDMotorControllers() {
    
  }
  
  /* Swerve Subsystem */
  public final WPI_TalonFX leftFrontSteerMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_FRONT_STEER_CAN_ID);
  public final WPI_TalonFX leftFrontWheelMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_FRONT_WHEEL_CAN_ID);
  public final WPI_TalonFX rightFrontSteerMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_FRONT_STEER_CAN_ID);
  public final WPI_TalonFX rightFrontWheelMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_FRONT_WHEEL_CAN_ID);
  public final WPI_TalonFX leftBackSteerMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_BACK_STEER_CAN_ID);
  public final WPI_TalonFX leftBackWheelMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_BACK_WHEEL_CAN_ID);
  public final WPI_TalonFX rightBackSteerMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_BACK_STEER_CAN_ID);
  public final WPI_TalonFX rightBackWheelMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_BACK_WHEEL_CAN_ID);
  public final CANCoder leftFrontEncoder = new CANCoder(SwerveDriveConstants.LEFT_FRONT_STEER_CAN_ENCODER_ID);
  public final CANCoder rightFrontEncoder = new CANCoder(SwerveDriveConstants.RIGHT_FRONT_STEER_CAN_ENCODER_ID);
  public final CANCoder leftBackEncoder = new CANCoder(SwerveDriveConstants.LEFT_BACK_STEER_CAN_ENCODER_ID);
  public final CANCoder rightBackEncoder = new CANCoder(SwerveDriveConstants.RIGHT_BACK_STEER_CAN_ENCODER_ID);
  public final WPI_PigeonIMU gyro = new WPI_PigeonIMU(SwerveDriveConstants.GYRO_ID);

  public SwerveModule leftFront;
  public SwerveModule leftBack;
  public SwerveModule rightFront;
  public SwerveModule rightBack;

  void configureSwerveMotorControllers() {

    leftFrontSteerMotor.configFactoryDefault();
    leftFrontWheelMotor.configFactoryDefault();
    rightFrontSteerMotor.configFactoryDefault();
    rightFrontWheelMotor.configFactoryDefault();
    leftBackSteerMotor.configFactoryDefault();
    leftBackWheelMotor.configFactoryDefault();
    rightBackSteerMotor.configFactoryDefault();
    rightBackWheelMotor.configFactoryDefault();

    leftFrontSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftFrontWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

    leftFrontWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftFrontSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

    leftFrontWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftFrontSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

    leftFrontSteerMotor.setNeutralMode(NeutralMode.Brake);
    leftFrontWheelMotor.setNeutralMode(NeutralMode.Brake);//Coast
    rightFrontSteerMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontWheelMotor.setNeutralMode(NeutralMode.Brake);//Coast
    leftBackSteerMotor.setNeutralMode(NeutralMode.Brake);
    leftBackWheelMotor.setNeutralMode(NeutralMode.Brake);//Coast
    rightBackSteerMotor.setNeutralMode(NeutralMode.Brake);
    rightBackWheelMotor.setNeutralMode(NeutralMode.Brake);//Coast

    leftFront = new SwerveModule(leftFrontWheelMotor, leftFrontSteerMotor, leftFrontEncoder, SwerveDriveConstants.LEFT_FRONT_ENCODER_OFFSET);
    leftBack = new SwerveModule(leftBackWheelMotor, leftBackSteerMotor, leftBackEncoder, SwerveDriveConstants.LEFT_BACK_ENCODER_OFFSET);
    rightFront = new SwerveModule(rightFrontWheelMotor, rightFrontSteerMotor, rightFrontEncoder, SwerveDriveConstants.RIGHT_FRONT_ENCODER_OFFSET);
    rightBack = new SwerveModule(rightBackWheelMotor, rightBackSteerMotor, rightBackEncoder, SwerveDriveConstants.RIGHT_BACK_ENCODER_OFFSET);

    // config cancoder as remote encoder for swerve steer motors
    //leftFrontSteerMotor.configRemoteFeedbackFilter(leftFrontEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    //leftBackSteerMotor.configRemoteFeedbackFilter(leftBackEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    //rightFrontSteerMotor.configRemoteFeedbackFilter(rightFrontEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    //rightBackSteerMotor.configRemoteFeedbackFilter(rightBackEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  }

}
