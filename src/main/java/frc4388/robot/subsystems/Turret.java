// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import javax.naming.ldap.Control;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.Gains;

public class Turret<CANSparkMax, CANDigitalInput> extends SubsystemBase {
  private static final String turretMotor = null;
  /** Creates a new Turret. */
  public BoomBoom m_boomBoomSubsystem;
  public SwerveDrive m_sDriveSubsystem;

  public CANSparkMax m_boomBoomRotateMotor = new CANSparkMax(ShooterConstants.SHOOTER_ROTATE_ID, MotorType.kBrushless);
  public static Gains m_shooterTGains = ShooterConstants.SHOOTER_TURRET_GAINS;
  CANDigitalInput m_boomBoomRightLimit, m_boomBoomLeftLimit;
  public Gyro m_turretGyro;

  public double m_targetDistance = 0;

  public boolean m_isAimReady = false;

  Controller m_boomBoomRotatePIDController = m_boomBoomRotateMotor.getPIDController();
  public CANCoder m_boomBoomRotateEncoder = m_boomBoomRotateMotor.getEncoder();
  
  
  //Variables
  public <m_boomBoomRotateMotor> Turret() {

    Object IdleMode;
    m_boomBoomRotateMotor.setIdleMode(IdleMode.kBrake);
    boolean enableLimitSwitch = true;
    m_turretGyro = getGyroInterface();
    m_boomBoomLeftLimit = m_boomBoomRotateMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_boomBoomRightLimit = m_boomBoomRotateMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_boomBoomRightLimit.enableLimitSwitch;
    m_boomBoomLeftLimit.enableLimitSwitch;

    m_boomBoomRotateMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_boomBoomRotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_boomBoomRotateMotor.setSoftLimit(SoftLimitDirection.kForward, ShooterConstants.TURRET_RIGHT_SOFT_LIMIT);
    boolean setInverted = false;
    m_boomBoomRotateMotor.setInverted;
    Object turretMotor;
    Object m_turretMotor = turretMotor;
  }

  private Gyro getGyroInterface() {
    return null;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Angle Raw", getboomBoomRotatePosition());

    SmartDashboard.putData("Turret Angle", (Sendable) m_turretGyro);

    SmartDashboard.putBoolean("Turret Aimed", m_isAimReady);
    // This method will be called once per scheduler run
  }

  public void passRequiredSubsystem( BoomBoom subsystem0, SwerveDrive subsystem1){
    m_boomBoomSubsystem = subsystem0;
    m_sDriveSubsystem = subsystem1;
  }

  public void runShooterWithInput(double input) {
    m_boomBoomRotateMotor.set(input*ShooterConstants.TURRET_SPEED_MULTIPLIER);
  }

  public void runshooterRotatePID(double targetAngle) {
  m_boomBoomRotatePIDController.setP(m_shooterTGains.m_kP);
  m_boomBoomRotatePIDController.setI(m_shooterTGains.m_kI);
  m_boomBoomRotatePIDController.setD(m_shooterTGains.m_kD);
  m_boomBoomRotatePIDController.setF(m_shooterTGains.m_kF);
  m_boomBoomRotatePIDController.setIZone(m_shooterTGains.m_kIzone);
  m_boomBoomRotatePIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN, m_shooterTGains.m_kPeakOutput);

  targetAngle = targetAngle/ShooterConstants.DEGREES_PER_ROT;

  m_boomBoomRotatePIDController.setReference(targetAngle,ControlType.kPosition);
  }

  public void resetGyroShooterRotate()
  {
    m_boomBoomRotateEncoder.setPosition(0);
  }

  public double getboomBoomRotatePosition()
  {
    return m_boomBoomRotateEncoder.getPosition();
  }

  public double getAnglePositionDegrees() {
    return (m_boomBoomRotateEncoder.getPosition() - ShooterConstants.TURRET_MOTOR_POS_AT_ZERO_ROT) * 360/ShooterConstants. TURRET_MOTOR_ROTS_PER_ROT;
  }
  
  public void turnWithJoystick (double input)
  {
    turretMotor.set(input);

  }

  //function turnWithJoystick(double input)
  //   motor.set(input)
}



/** TODO
* setPosition function
* Limit switches
**/