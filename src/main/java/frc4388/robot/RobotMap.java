// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.Constants.SerializerConstants;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.SwerveModule;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {

  public RobotMap() {
    // configureLEDMotorControllers();
    configureSwerveMotorControllers();
    configureShooterMotorControllers();
    configureIntakeMotors();
    configureExtenderMotors();
    configureSerializerMotors();
    configureStorageMotors();
  }

  /* LED Subsystem */
//   public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);

//   void configureLEDMotorControllers() {}

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

    leftFrontSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftFrontWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackSteerMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackWheelMotor.configOpenloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);

    leftFrontWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftFrontSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackSteerMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackWheelMotor.configClosedloopRamp(SwerveDriveConstants.OPEN_LOOP_RAMP_RATE,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);

    leftFrontWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftFrontSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackSteerMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackWheelMotor.configNeutralDeadband(SwerveDriveConstants.NEUTRAL_DEADBAND,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);

    NeutralMode mode = NeutralMode.Coast;
    leftFrontSteerMotor.setNeutralMode(mode);
    leftFrontWheelMotor.setNeutralMode(mode);// Coast
    rightFrontSteerMotor.setNeutralMode(mode);
    rightFrontWheelMotor.setNeutralMode(mode);// Coast
    leftBackSteerMotor.setNeutralMode(mode);
    leftBackWheelMotor.setNeutralMode(mode);// Coast
    rightBackSteerMotor.setNeutralMode(mode);
    rightBackWheelMotor.setNeutralMode(mode);// Coast

    // current limits
    
    leftFrontSteerMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_STEER);
    rightFrontSteerMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_STEER);
    leftBackSteerMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_STEER);
    rightBackSteerMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_STEER);
    
    leftFrontWheelMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_WHEEL);
    rightFrontWheelMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_WHEEL);
    leftBackWheelMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_WHEEL);
    rightBackWheelMotor.configSupplyCurrentLimit(SwerveDriveConstants.SUPPLY_CURRENT_LIMIT_CONFIG_WHEEL);

    leftFrontSteerMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_STEER);
    rightFrontSteerMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_STEER);
    leftBackSteerMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_STEER);
    rightBackSteerMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_STEER);
    
    leftFrontWheelMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_WHEEL);
    rightFrontWheelMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_WHEEL);
    leftBackWheelMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_WHEEL);
    rightBackWheelMotor.configStatorCurrentLimit(SwerveDriveConstants.STATOR_CURRENT_LIMIT_CONFIG_WHEEL);

    leftFront = new SwerveModule(leftFrontWheelMotor, leftFrontSteerMotor, leftFrontEncoder,
        SwerveDriveConstants.LEFT_FRONT_ENCODER_OFFSET);
    leftBack = new SwerveModule(leftBackWheelMotor, leftBackSteerMotor, leftBackEncoder,
        SwerveDriveConstants.LEFT_BACK_ENCODER_OFFSET);
    rightFront = new SwerveModule(rightFrontWheelMotor, rightFrontSteerMotor, rightFrontEncoder,
        SwerveDriveConstants.RIGHT_FRONT_ENCODER_OFFSET);
    rightBack = new SwerveModule(rightBackWheelMotor, rightBackSteerMotor, rightBackEncoder,
        SwerveDriveConstants.RIGHT_BACK_ENCODER_OFFSET);

    // config cancoder as remote encoder for swerve steer motors
    leftFrontSteerMotor.configRemoteFeedbackFilter(leftFrontEncoder.getDeviceID(),
        RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    leftBackSteerMotor.configRemoteFeedbackFilter(leftBackEncoder.getDeviceID(),
        RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightFrontSteerMotor.configRemoteFeedbackFilter(rightFrontEncoder.getDeviceID(),
        RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    rightBackSteerMotor.configRemoteFeedbackFilter(rightBackEncoder.getDeviceID(),
        RemoteSensorSource.CANCoder, SwerveDriveConstants.REMOTE_0,
        SwerveDriveConstants.SWERVE_TIMEOUT_MS);
}

//   // Shooter Config
//   /* Boom Boom Subsystem */
  public final WPI_TalonFX shooterFalconLeft = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_LEFT_CAN_ID);
  public final WPI_TalonFX shooterFalconRight = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_RIGHT_CAN_ID);

//   // turret subsystem
  public final CANSparkMax shooterTurret = new CANSparkMax(ShooterConstants.TURRET_MOTOR_CAN_ID, MotorType.kBrushless);

  // hood subsystem
  public CANSparkMax angleAdjusterMotor = new CANSparkMax(ShooterConstants.SHOOTER_ANGLE_ADJUST_ID, MotorType.kBrushless);

  // Create motor CANSparkMax
  void configureShooterMotorControllers() {

    // LEFT FALCON
    shooterFalconLeft.configFactoryDefault();
    shooterFalconLeft.setNeutralMode(NeutralMode.Coast);
    shooterFalconLeft.setInverted(true);
    shooterFalconLeft.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configClosedloopRamp(0.75, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configPeakOutputReverse(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX,
        ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configClosedLoopPeriod(0, ShooterConstants.CLOSED_LOOP_TIME_MS,
        ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG_SHOOTER,
        ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconLeft.configStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT_CONFIG_SHOOTER,
        ShooterConstants.SHOOTER_TIMEOUT_MS);

    // RIGHT FALCON
    shooterFalconRight.setInverted(false);
    shooterFalconRight.setNeutralMode(NeutralMode.Coast);
    shooterFalconRight.configFactoryDefault();
    shooterFalconRight.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configClosedloopRamp(0.75, ShooterConstants.SHOOTER_TIMEOUT_MS);
    // m_shooterFalconRight.configPeakOutputForward(0,
    // ShooterConstants.SHOOTER_TIMEOUT_MS);(comment it in if necessary)
    shooterFalconRight.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX,
        ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configClosedLoopPeriod(0, ShooterConstants.CLOSED_LOOP_TIME_MS,
        ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG_SHOOTER,
        ShooterConstants.SHOOTER_TIMEOUT_MS);
    shooterFalconRight.configStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT_CONFIG_SHOOTER,
        ShooterConstants.SHOOTER_TIMEOUT_MS);

    shooterFalconRight.follow(shooterFalconLeft);
  }

//     // /* Turret Subsytem */
    // shooterFalconRight.configStatorCurrentLimit(new
    // StatorCurrentLimitConfiguration(true, 6, 9, 4.2)); // TODO: dont pull numbers
    // out of our ass anymore
    // shooterFalconLeft.configSupplyCurrentLimit(new
    // SupplyCurrentLimitConfiguration(true, 12, 13, 0.4)); // TODO: dont pull
//     // numbers out of our ass anymore

//     // hood subsystem
//     angleAdjusterMotor.restoreFactoryDefaults();
//     angleAdjusterMotor.setIdleMode(IdleMode.kBrake);
//     angleAdjusterMotor.setInverted(true);
//   }


       
  /* Serializer Subsystem */
  public final CANSparkMax serializerBelt = new CANSparkMax(SerializerConstants.SERIALIZER_BELT, MotorType.kBrushless);
  public final DigitalInput serializerBeam = new DigitalInput(SerializerConstants.SERIALIZER_BELT_BEAM);
  
  /* Intake Subsytem */
  public final WPI_TalonFX intakeMotor = new WPI_TalonFX(IntakeConstants.INTAKE_MOTOR);
  public final CANSparkMax extenderMotor = new CANSparkMax(IntakeConstants.EXTENDER_MOTOR, MotorType.kBrushless);

  void configureIntakeMotors() {
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(false);
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    intakeMotor.configSupplyCurrentLimit(IntakeConstants.SUPPLY_CURRENT_LIMIT_CONFIG_INTAKE);
    intakeMotor.configStatorCurrentLimit(IntakeConstants.STATOR_CURRENT_LIMIT_CONFIG_INTAKE);
  }

  void configureExtenderMotors() {
      extenderMotor.restoreFactoryDefaults();
      extenderMotor.setInverted(true);
      extenderMotor.setIdleMode(IdleMode.kBrake);
  }

    void configureSerializerMotors() {
        serializerBelt.restoreFactoryDefaults();
    }

  /* Storage Subsystem */
    public final CANSparkMax storageMotor = new CANSparkMax(StorageConstants.STORAGE_CAN_ID, MotorType.kBrushless);
//   public final DigitalInput beamShooter = new DigitalInput(StorageConstants.BEAM_SENSOR_SHOOTER);
//   public final DigitalInput beamIntake = new DigitalInput(StorageConstants.BEAM_SENSOR_INTAKE);

    void configureStorageMotors() {
        storageMotor.restoreFactoryDefaults();
    }

}
