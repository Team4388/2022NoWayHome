// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.Gains;

public class Hood extends SubsystemBase {
  public BoomBoom m_shooterSubsystem;
  
  public CANSparkMax m_angleAdjustMotor = new CANSparkMax(ShooterConstants.SHOOTER_ANGLE_ADJUST_ID, MotorType.kBrushless);
  
  public static Gains m_angleGains = ShooterConstants.SHOOTER_ANGLE_GAINS;
  public CANEncoder m_angleEncoder = m_angleAdjustMotor.getEncoder();

  public SparkMaxPIDController m_angleAdjustPIDController = m_angleAdjustMotor.getPIDController();

  public boolean m_isHoodReady = false;

public double m_fireAngle;
  
  /** Creates a new Hood. */
  public Hood() {
    m_angleAdjusterMotor.setIdleMode(IdleMode.kBrake);

    m_hoodUpLimit = m_angleAdjusterMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_hoodDownLimit = m_angleAdjusterMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_hoodUpLimit.enableLimitSwitch(true);
    m_hoodDownLimit.enableLimitSwitch(true);
  }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runAngleAdjustPID(double targetAngle) 
  {
    //Set PID Coefficients
    m_angleAdjustPIDController.setP(m_angleAdjusterGains.m_kP);
    m_angleAdjustPIDController.setI(m_angleAdjusterGains.m_kI);
    m_angleAdjusterPIDController.setD(m_angleAdjusterGains.m_kD);
    m_angleAdjusterPIDController.setIZone(m_angleAdjustGains.m_IZone);
    m_angleAdjusterPIDController.setFF(m_angleAdjustGains.m_kF);
    m_angleAdjusterPIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN, m_angleAdjusterGains.m_kPeakOutput);

    m_angleAdjustPIDController.setReference(targetAngle, ControlType.kPosition);
  }
  

  public void runHood(double input) {
    m_angleAdjusterMotor.set(input);
  }

  public void resetGyroAngleAdj(){
    m_angleEncoder.setPosition(0);
  }

  public double getAnglePosition(){
    return m_angleEncoder.getPosition();
  }

  public double getAnglePositionDegrees(){
    return ((m_angleEncoder.getPosition() - ShooterConstants.HOOD_MOTOR_POS_AT_ZERO_ROT) * 360/ShooterConstants.HOOD_MOTOR_ROTS_PER_ROT) - 90;
  }
}
