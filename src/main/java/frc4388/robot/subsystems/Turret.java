// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.Gains;

public class Turret extends SubsystemBase {

  /** Creates a new Turret. */
  public BoomBoom m_boomBoomSubsystem;
  public SwerveDrive m_sDriveSubsystem;

  public CANSparkMax m_boomBoomRotateMotor;// = new CANSparkMax(ShooterConstants.SHOOTER_ROTATE_ID,
                                           // MotorType.kBrushless);
  public static Gains m_shooterTGains = ShooterConstants.SHOOTER_TURRET_GAINS;
  SparkMaxLimitSwitch m_boomBoomRightLimit, m_boomBoomLeftLimit;

  SparkMaxPIDController m_boomBoomRotatePIDController;
  public RelativeEncoder m_boomBoomRotateEncoder;

  public Turret(CANSparkMax boomBoomRotateMotor) {

    m_boomBoomRotateMotor = boomBoomRotateMotor;
    m_boomBoomRotatePIDController = m_boomBoomRotateMotor.getPIDController();
    m_boomBoomRotateEncoder = m_boomBoomRotateMotor.getEncoder();

    m_boomBoomRightLimit = m_boomBoomRotateMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_boomBoomLeftLimit = m_boomBoomRotateMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    setTurretLimitSwitches(false);
    // SmartDashboard.putBoolean("Right Limit Switch Enabled", m_boomBoomRightLimit.isLimitSwitchEnabled());
    // SmartDashboard.putBoolean("Left Limit Switch Enabled", m_boomBoomLeftLimit.isLimitSwitchEnabled());

    m_boomBoomRotateMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ShooterConstants.TURRET_FORWARD_LIMIT);
    m_boomBoomRotateMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ShooterConstants.TURRET_REVERSE_LIMIT);
    setTurretSoftLimits(true);

    setTurretPIDGains();
  }
  
  /**
   * Set gains for turret PIDs.
   */
  public void setTurretPIDGains() {
    m_boomBoomRotatePIDController.setP(m_shooterTGains.kP);
    m_boomBoomRotatePIDController.setI(m_shooterTGains.kI);
    m_boomBoomRotatePIDController.setD(m_shooterTGains.kD);
    m_boomBoomRotatePIDController.setFF(m_shooterTGains.kF);
    m_boomBoomRotatePIDController.setIZone(m_shooterTGains.kIzone);
    m_boomBoomRotatePIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN, m_shooterTGains.kPeakOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Angle Rotations", m_boomBoomRotateEncoder.getPosition());
    SmartDashboard.putNumber("Turret Angle Degrees", m_boomBoomRotateEncoder.getPosition() * ShooterConstants.TURRET_DEGREES_PER_ROT);
  }

  /**
   * Set status of turret motor soft limits.
   * @param set Boolean to set soft limits to.
   */
  public void setTurretSoftLimits(boolean set) {
    m_boomBoomRotateMotor.enableSoftLimit(SoftLimitDirection.kForward, set);
    m_boomBoomRotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, set);
  }

  /**
   * Set status of turret limit switches.
   * @param set Boolean to set limit switches to.
   */
  public void setTurretLimitSwitches(boolean set) {
    m_boomBoomRightLimit.enableLimitSwitch(set);
    m_boomBoomLeftLimit.enableLimitSwitch(set);
  }

  public void passRequiredSubsystem(BoomBoom subsystem0, SwerveDrive subsystem1) {
    m_boomBoomSubsystem = subsystem0;
    m_sDriveSubsystem = subsystem1;
  }

  public void runTurretWithInput(double input) {
    m_boomBoomRotateMotor.set(input * ShooterConstants.TURRET_SPEED_MULTIPLIER);
  }

  public void runShooterRotatePID(double targetAngle) {
    targetAngle = targetAngle / ShooterConstants.TURRET_DEGREES_PER_ROT;
    m_boomBoomRotatePIDController.setReference(targetAngle, ControlType.kPosition);
  }

  public void resetGyroShooterRotate() {
    m_boomBoomRotateEncoder.setPosition(0);
  }

  /**
   * Run a PID to go to the zero position.
   */
  public void gotoZero() {
    runShooterRotatePID(0);
  }

  /**
   * Run a PID to go to the midpoint position, between the two soft limits.
   */
  public void gotoMidpoint() {
    runShooterRotatePID(-44 * ShooterConstants.TURRET_DEGREES_PER_ROT);
  }

  public double getEncoderPosition() {
    return m_boomBoomRotateEncoder.getPosition();
  }

  public double getBoomBoomAngleDegrees() {
    return (m_boomBoomRotateEncoder.getPosition() - ShooterConstants.TURRET_MOTOR_POS_AT_ZERO_ROT) * 360
        / ShooterConstants.TURRET_MOTOR_ROTS_PER_ROT;
  } // TODO: does this method work?

  public double getCurrent(){
    return m_boomBoomRotateMotor.getOutputCurrent();
  }

}