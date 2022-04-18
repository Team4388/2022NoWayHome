// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc4388.robot.Constants.ClimberConstants;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.SerializerConstants;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.SwerveModule;
import frc4388.utility.shuffleboard.SendableCANSparkMax;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit testing and
 * modularization.
 */
public class RobotMap {
  // #region Swerve Subsystem
  public final WPI_TalonFX frontLeftSteerMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_FRONT_STEER_CAN_ID);
  public final WPI_TalonFX frontLeftDriveMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_FRONT_WHEEL_CAN_ID);
  public final WPI_TalonFX frontRightSteerMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_FRONT_STEER_CAN_ID);
  public final WPI_TalonFX frontRightDriveMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_FRONT_WHEEL_CAN_ID);
  public final WPI_TalonFX backLeftSteerMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_BACK_STEER_CAN_ID);
  public final WPI_TalonFX backLeftDriveMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_BACK_WHEEL_CAN_ID);
  public final WPI_TalonFX backRightSteerMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_BACK_STEER_CAN_ID);
  public final WPI_TalonFX backRightDriveMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_BACK_WHEEL_CAN_ID);
  public final CANCoder frontLeftEncoder = new CANCoder(SwerveDriveConstants.LEFT_FRONT_STEER_CAN_ENCODER_ID);
  public final CANCoder frontRightEncoder = new CANCoder(SwerveDriveConstants.RIGHT_FRONT_STEER_CAN_ENCODER_ID);
  public final CANCoder backLeftEncoder = new CANCoder(SwerveDriveConstants.LEFT_BACK_STEER_CAN_ENCODER_ID);
  public final CANCoder backRightEncoder = new CANCoder(SwerveDriveConstants.RIGHT_BACK_STEER_CAN_ENCODER_ID);

  public SwerveModule frontLeft;
  public SwerveModule frontRight;
  public SwerveModule backLeft;
  public SwerveModule backRight;

  public final WPI_Pigeon2 gyro = new WPI_Pigeon2(SwerveDriveConstants.GYRO_ID);
  // #endregion Swerve Subsystem

  // #region Extender Subsystem
  public final CANSparkMax extenderMotor = new SendableCANSparkMax(IntakeConstants.EXTENDER_MOTOR, MotorType.kBrushless);
  // #endregion Extender Subsystem

  // #region Intake Subsystem
  public final WPI_TalonFX intakeMotor = new WPI_TalonFX(IntakeConstants.INTAKE_MOTOR);
  // #endregion Intake Subsystem

  // #region Serializer Subsystem
  public final CANSparkMax serializerBelt = new SendableCANSparkMax(SerializerConstants.SERIALIZER_BELT, MotorType.kBrushless);
  // #endregion Serializer Subsystem

  // #region Storage Subsystem
  public final CANSparkMax storageMotor = new SendableCANSparkMax(StorageConstants.STORAGE_CAN_ID, MotorType.kBrushless);
  // #endregion Storage Subsystem

  // #region Shooter System
  // #region Boom Boom Subsystem
  public final WPI_TalonFX shooterFalconLeft = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_LEFT_CAN_ID);
  public final WPI_TalonFX shooterFalconRight = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_RIGHT_CAN_ID);
  // #endregion Boom Boom Subsystem

  // #region Turret Subsystem
  public final CANSparkMax shooterTurret = new SendableCANSparkMax(ShooterConstants.TURRET_MOTOR_CAN_ID, MotorType.kBrushless);
  // #endregion Turret Subsystem

  // #region Hood Subsystem
  public final CANSparkMax angleAdjusterMotor = new SendableCANSparkMax(ShooterConstants.SHOOTER_ANGLE_ADJUST_ID, MotorType.kBrushless);
  // #endregion Hood Subsystem
  // #endregion Shooter System

  // #region Climber Subsystem
  public final WPI_TalonFX elbow = new WPI_TalonFX(ClimberConstants.ELBOW_ID);
  // #endregion Climber Subsystem

  // #region Claws Subsystem
  public final Servo leftClaw = new Servo(1);
  public final Servo rightClaw = new Servo(2);
  // #endregion Claws Subsystem

  // #region LED Subsystem
  public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);
  // #endregion LED Subsystem

  public RobotMap() {
    configureSwerveMotorControllers();
    configureExtenderMotors();
    configureIntakeMotors();
    configureSerializerMotors();
    configureStorageMotors();
    configureShooterMotorControllers();
    configureClimberMotors();
    registerDevices();
  }

  void configureSwerveMotorControllers() {
    frontLeftSteerMotor.configFactoryDefault();
    frontLeftDriveMotor.configFactoryDefault();
    frontRightSteerMotor.configFactoryDefault();
    frontRightDriveMotor.configFactoryDefault();
    backLeftSteerMotor.configFactoryDefault();
    backLeftDriveMotor.configFactoryDefault();
    backRightSteerMotor.configFactoryDefault();
    backRightDriveMotor.configFactoryDefault();

    frontLeftSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    frontLeftDriveMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    frontRightSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    frontRightDriveMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backLeftSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backLeftDriveMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backRightSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backRightDriveMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

    frontLeftDriveMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    frontLeftSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    frontRightSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    frontRightDriveMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backLeftSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backLeftDriveMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backRightSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backRightDriveMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

    frontLeftDriveMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    frontLeftSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    frontRightSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    frontRightDriveMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backLeftSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backLeftDriveMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backRightSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backRightDriveMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

    frontLeftSteerMotor.setNeutralMode(NeutralMode.Brake);
    frontRightSteerMotor.setNeutralMode(NeutralMode.Brake);
    backLeftSteerMotor.setNeutralMode(NeutralMode.Brake);
    backRightSteerMotor.setNeutralMode(NeutralMode.Brake);

    frontLeftDriveMotor.setNeutralMode(NeutralMode.Coast);
    frontRightDriveMotor.setNeutralMode(NeutralMode.Coast);
    backLeftDriveMotor.setNeutralMode(NeutralMode.Coast);
    backRightDriveMotor.setNeutralMode(NeutralMode.Coast);

    // current limits
    frontLeftSteerMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_STEER);
    frontRightSteerMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_STEER);
    backLeftSteerMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_STEER);
    backRightSteerMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_STEER);

    frontLeftDriveMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_WHEEL);
    frontRightDriveMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_WHEEL);
    backLeftDriveMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_WHEEL);
    backRightDriveMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_WHEEL);

    frontLeftSteerMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_STEER);
    frontRightSteerMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_STEER);
    backLeftSteerMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_STEER);
    backRightSteerMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_STEER);

    frontLeftDriveMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_WHEEL);
    frontRightDriveMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_WHEEL);
    backLeftDriveMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_WHEEL);
    backRightDriveMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_WHEEL);

    frontLeft = new SwerveModule(frontLeftDriveMotor, frontLeftSteerMotor, frontLeftEncoder, SwerveDriveConstants.LEFT_FRONT_ENCODER_OFFSET);
    frontRight = new SwerveModule(frontRightDriveMotor, frontRightSteerMotor, frontRightEncoder, SwerveDriveConstants.RIGHT_FRONT_ENCODER_OFFSET);
    backLeft = new SwerveModule(backLeftDriveMotor, backLeftSteerMotor, backLeftEncoder, SwerveDriveConstants.LEFT_BACK_ENCODER_OFFSET);
    backRight = new SwerveModule(backRightDriveMotor, backRightSteerMotor, backRightEncoder, SwerveDriveConstants.RIGHT_BACK_ENCODER_OFFSET);

    // config cancoder as remote encoder for swerve steer motors
    frontLeftSteerMotor.configRemoteFeedbackFilter(frontLeftEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backLeftSteerMotor.configRemoteFeedbackFilter(backLeftEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    frontRightSteerMotor.configRemoteFeedbackFilter(frontRightEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    backRightSteerMotor.configRemoteFeedbackFilter(backRightEncoder.getDeviceID(), RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
  }

  void configureExtenderMotors() {
    extenderMotor.restoreFactoryDefaults();
    extenderMotor.setInverted(true);
    extenderMotor.setIdleMode(IdleMode.kBrake);
  }

  void configureIntakeMotors() {
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(false);
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    intakeMotor.configSupplyCurrentLimit(IntakeConstants.SUPPLY_CURRENT_LIMIT_CONFIG_INTAKE);
    intakeMotor.configStatorCurrentLimit(IntakeConstants.STATOR_CURRENT_LIMIT_CONFIG_INTAKE);
  }

  void configureSerializerMotors() {
    serializerBelt.restoreFactoryDefaults();
  }

  void configureStorageMotors() {
    storageMotor.restoreFactoryDefaults();
    storageMotor.setIdleMode(IdleMode.kCoast);
  }

  void configureShooterMotorControllers() {
    // Boom Boom Subsystem
    shooterFalconLeft.configFactoryDefault();
    shooterFalconLeft.setNeutralMode(NeutralMode.Coast);
    shooterFalconLeft.setInverted(true);
    shooterFalconLeft.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configClosedloopRamp(0.75, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configPeakOutputReverse(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configClosedLoopPeriod(0, ShooterConstants.CLOSED_LOOP_TIME_MS, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG_SHOOTER, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT_CONFIG_SHOOTER, ShooterConstants.SHOOTER_TIMEOUT_MS);

    shooterFalconRight.configFactoryDefault();
    shooterFalconRight.setNeutralMode(NeutralMode.Coast);
    shooterFalconRight.setInverted(false);
    shooterFalconRight.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configClosedloopRamp(0.75, ShooterConstants.SHOOTER_TIMEOUT_MS);
    // shooterFalconRight.configPeakOutputForward(0, ShooterConstants.SHOOTER_TIMEOUT_MS); //(comment it in if necessary)
    shooterFalconRight.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configClosedLoopPeriod(0, ShooterConstants.CLOSED_LOOP_TIME_MS, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG_SHOOTER, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT_CONFIG_SHOOTER, ShooterConstants.SHOOTER_TIMEOUT_MS);

    shooterFalconRight.follow(shooterFalconLeft);

    // Turret Subsystem
    shooterTurret.restoreFactoryDefaults();
    shooterTurret.setIdleMode(IdleMode.kBrake);
    shooterTurret.setInverted(true);

    // Hood Subsystem
    angleAdjusterMotor.restoreFactoryDefaults();
    angleAdjusterMotor.setIdleMode(IdleMode.kBrake);
    angleAdjusterMotor.setInverted(true);
  }

  void configureClimberMotors() {
    elbow.configFactoryDefault();
    elbow.setNeutralMode(NeutralMode.Brake);
  }

  private void registerDevices() {
    SendableRegistry.setName(frontLeftSteerMotor, "SwerveDrive", "Left Front Steer Motor");
    SendableRegistry.setName(frontLeftDriveMotor, "SwerveDrive", "Left Front Drive Motor");
    SendableRegistry.setName(frontRightSteerMotor, "SwerveDrive", "Right Front Steer Motor");
    SendableRegistry.setName(frontRightDriveMotor, "SwerveDrive", "Right Front Drive Motor");
    SendableRegistry.setName(backLeftSteerMotor, "SwerveDrive", "Left Back Steer Motor");
    SendableRegistry.setName(backLeftDriveMotor, "SwerveDrive", "Left Back Drive Motor");
    SendableRegistry.setName(backRightSteerMotor, "SwerveDrive", "Right Back Steer Motor");
    SendableRegistry.setName(backRightDriveMotor, "SwerveDrive", "Right Back Drive Motor");

    SendableRegistry.setName(intakeMotor, "Intake", "Intake Motor");
    SendableRegistry.setName((SendableCANSparkMax) extenderMotor, "Intake", "Extender Motor");
    SendableRegistry.setName((SendableCANSparkMax) serializerBelt, "Intake", "Serializer Belt");
    SendableRegistry.setName((SendableCANSparkMax) storageMotor, "Intake", "Storage Motor");

    SendableRegistry.setName(shooterFalconLeft, "Shooter", "BoomBoom Left Motor");
    SendableRegistry.setName(shooterFalconRight, "Shooter", "BoomBoom Right Motor");
    SendableRegistry.setName((SendableCANSparkMax) shooterTurret, "Shooter", "Turret");
    SendableRegistry.setName((SendableCANSparkMax) angleAdjusterMotor, "Shooter", "Hood");

    SendableRegistry.setName(elbow, "Climber", "Elbow");
    SendableRegistry.setName(leftClaw, "Climber", "Left Claw");
    SendableRegistry.setName(rightClaw, "Climber", "Right Claw");
  }
}
