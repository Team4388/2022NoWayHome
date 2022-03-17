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
import frc4388.robot.commands.ShooterCommands.Shoot;
import frc4388.utility.Gains;

public class Turret extends SubsystemBase {

  /** Creates a new Turret. */
  public BoomBoom m_boomBoomSubsystem;
  public SwerveDrive m_sDriveSubsystem;

  public CANSparkMax m_boomBoomRotateMotor;// = new CANSparkMax(ShooterConstants.SHOOTER_ROTATE_ID,
                                           // MotorType.kBrushless);
  public static Gains m_shooterTGains = ShooterConstants.SHOOTER_TURRET_GAINS;

  public SparkMaxPIDController m_boomBoomRotatePIDController;
  public RelativeEncoder m_boomBoomRotateEncoder;

  SparkMaxLimitSwitch m_boomBoomLeftLimit;
  SparkMaxLimitSwitch m_boomBoomRightLimit;

  boolean hasLeftSwitchChanged = false;
  boolean hasRightSwitchChanged = false;

  boolean leftPrevState = false;
  boolean rightPrevState = false;
  boolean leftState;
  boolean rightState;

  long leftCurrentTime;
  long rightCurrentTime;
  long leftElapsedTime;
  long rightElapsedTime;

  public Turret(CANSparkMax boomBoomRotateMotor) {

    m_boomBoomRotateMotor = boomBoomRotateMotor;
    m_boomBoomRotatePIDController = m_boomBoomRotateMotor.getPIDController();
    m_boomBoomRotateEncoder = m_boomBoomRotateMotor.getEncoder();

    m_boomBoomLeftLimit = m_boomBoomRotateMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_boomBoomRightLimit = m_boomBoomRotateMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    // m_boomBoomLeftLimit.enableLimitSwitch(true);
    // m_boomBoomRightLimit.enableLimitSwitch(true);
    setTurretLimitSwitches(true);

    m_boomBoomRotateMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ShooterConstants.TURRET_FORWARD_SOFT_LIMIT);
    m_boomBoomRotateMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ShooterConstants.TURRET_REVERSE_SOFT_LIMIT);
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
    
    SmartDashboard.putBoolean("Right Limit Switch Enabled", m_boomBoomRightLimit.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Left Limit Switch Enabled", m_boomBoomLeftLimit.isLimitSwitchEnabled());
    
    SmartDashboard.putNumber("Turret Angle Rotations", m_boomBoomRotateEncoder.getPosition());
    SmartDashboard.putNumber("Turret Angle Degrees", m_boomBoomRotateEncoder.getPosition() * ShooterConstants.TURRET_DEGREES_PER_ROT);
    SmartDashboard.putBoolean("Left Limit Switch Pressed", m_boomBoomLeftLimit.isPressed());
    SmartDashboard.putBoolean("Right Limit Switch Pressed", m_boomBoomRightLimit.isPressed());

    // limit switch annoying time thing
    leftState = m_boomBoomLeftLimit.isPressed();
    rightState = m_boomBoomRightLimit.isPressed();

    hasLeftSwitchChanged = (leftState != leftPrevState);
    hasRightSwitchChanged = (rightState != rightPrevState);

    if (leftState && hasLeftSwitchChanged) {
      leftCurrentTime = System.currentTimeMillis();
      leftElapsedTime = 0;
    }
    
    if (rightState && hasRightSwitchChanged) {
      rightCurrentTime = System.currentTimeMillis();
      rightElapsedTime = 0;
    }

    if (leftState && !hasLeftSwitchChanged) {
      leftElapsedTime = System.currentTimeMillis() - leftCurrentTime;
    }
    
    if (rightState && !hasRightSwitchChanged) {
      rightElapsedTime = System.currentTimeMillis() - rightCurrentTime;
    }

    if (leftState && (leftElapsedTime > 500)) {
      m_boomBoomRotateEncoder.setPosition(ShooterConstants.TURRET_FORWARD_HARD_LIMIT);// -95/*ShooterConstants.TURRET_FORWARD_SOFT_LIMIT - 2*/);
    }
    if (rightState && (rightElapsedTime > 500)) {
      m_boomBoomRotateEncoder.setPosition(ShooterConstants.TURRET_REVERSE_HARD_LIMIT);// 0/*ShooterConstants.TURRET_REVERSE_LIMIT + 2*/);
    }

    leftPrevState = leftState;
    rightPrevState = rightState;
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
  /**
   * Move the turret with an input
   * @param input from -1.0 to 1.0, positive is clockwise
   */
  public void runTurretWithInput(double input) {
    m_boomBoomRotateMotor.set(input * ShooterConstants.TURRET_SPEED_MULTIPLIER * 0.5);
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
    return (getEncoderPosition() * ShooterConstants.TURRET_DEGREES_PER_ROT);
  }

  public double getCurrent(){
    return m_boomBoomRotateMotor.getOutputCurrent();
  }

}