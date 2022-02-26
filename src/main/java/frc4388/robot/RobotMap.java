// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.SerializerConstants;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  
  /* Swerve Subsystem 
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
      
  void configureSwerveMotorControllers() {
    leftFrontEncoder.configMagnetOffset(SwerveDriveConstants.LEFT_BACK_ENCODER_OFFSET);
    rightFrontEncoder.configMagnetOffset(SwerveDriveConstants.RIGHT_FRONT_ENCODER_OFFSET);
    leftBackEncoder.configMagnetOffset(SwerveDriveConstants.LEFT_BACK_ENCODER_OFFSET);
    rightBackEncoder.configMagnetOffset(SwerveDriveConstants.RIGHT_BACK_ENCODER_OFFSET);

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

    // config cancoder as remote encoder for swerve steer motors
    //leftFrontSteerMotor.configRemoteFeedbackFilter(leftFrontEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    //leftBackSteerMotor.configRemoteFeedbackFilter(leftBackEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    //rightFrontSteerMotor.configRemoteFeedbackFilter(rightFrontEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    //rightBackSteerMotor.configRemoteFeedbackFilter(rightBackEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  }

  /* Serializer Subsystem */
  public final CANSparkMax serializerBelt = new CANSparkMax(SerializerConstants.SERIALIZER_BELT, MotorType.kBrushless);
  public final CANSparkMax serializerShooterBelt = new CANSparkMax(SerializerConstants.SERIALIZER_SHOOTER_BELT, MotorType.kBrushless);
  public final DigitalInput serializerBeam = new DigitalInput(SerializerConstants.SERIALIZER_BELT_BEAM);
  
  /* Intake Subsytem */
  public final WPI_TalonFX intakeMotor = new WPI_TalonFX(IntakeConstants.INTAKE_MOTOR);
  public final CANSparkMax extenderMotor = new CANSparkMax(IntakeConstants.EXTENDER_MOTOR, MotorType.kBrushless);

  /* Storage Subsystem */
  public final CANSparkMax storageMotor = new CANSparkMax(StorageConstants.STORAGE_CAN_ID, MotorType.kBrushless);
  public final DigitalInput beamShooter = new DigitalInput(StorageConstants.BEAM_SENSOR_SHOOTER);
  public final DigitalInput beamIntake = new DigitalInput(StorageConstants.BEAM_SENSOR_INTAKE);
}
