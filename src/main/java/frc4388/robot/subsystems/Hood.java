// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import javax.sql.rowset.WebRowSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.Gains;

public class Hood extends SubsystemBase {
  public BoomBoom m_shooterSubsystem;
  
  public CANSparkMax m_angleAdjusterMotor = new CANSparkMax(ShooterConstants.SHOOTER_ANGLE_ADJUST_ID, MotorType.kBrushless);
  public SparkMaxLimitSwitch m_hoodUpLimitSwitch;
  public SparkMaxLimitSwitch m_hoodDownLimitSwitch;
  public static Gains m_angleAdjusterGains = ShooterConstants.SHOOTER_ANGLE_GAINS;
  public RelativeEncoder m_angleEncoder = m_angleAdjusterMotor.getEncoder();

  public SparkMaxPIDController m_angleAdjusterPIDController = m_angleAdjusterMotor.getPIDController();
  
  
  public boolean m_isHoodReady = false;

public double m_fireAngle;
  

  /** Creates a new Hood. */
  public Hood() {
     m_angleAdjusterMotor.setIdleMode(IdleMode.kBrake);

    m_hoodUpLimitSwitch = m_angleAdjusterMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_hoodDownLimitSwitch = m_angleAdjusterMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_hoodUpLimitSwitch.enableLimitSwitch(true);
    m_hoodDownLimitSwitch.enableLimitSwitch(true);

    m_angleAdjusterMotor.setSoftLimit(SoftLimitDirection.kForward, ShooterConstants.HOOD_FORWARD_LIMIT);
    m_angleAdjusterMotor.setSoftLimit(SoftLimitDirection.kReverse, ShooterConstants.HOOD_REVERSE_LIMIT);
    setHoodSoftLimits(true);
  }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /**
   * Set status of hood motor soft limits.
   * @param set Boolean to set soft limits to.
   */
  public void setHoodSoftLimits(boolean set) {
    m_angleAdjusterMotor.enableSoftLimit(SoftLimitDirection.kForward, set);
    m_angleAdjusterMotor.enableSoftLimit(SoftLimitDirection.kReverse, set);
  }

  public void runAngleAdjustPID(double targetAngle) 
  {
    //Set PID Coefficients
    m_angleAdjusterPIDController.setP(m_angleAdjusterGains.kP);
    m_angleAdjusterPIDController.setI(m_angleAdjusterGains.kI);
    m_angleAdjusterPIDController.setD(m_angleAdjusterGains.kD);
    m_angleAdjusterPIDController.setIZone(m_angleAdjusterGains.kIzone);
    m_angleAdjusterPIDController.setFF(m_angleAdjusterGains.kF);
    m_angleAdjusterPIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN, m_angleAdjusterGains.kPeakOutput);

    m_angleAdjusterPIDController.setReference(targetAngle, ControlType.kPosition);
  }
  

  public void runHood(double input) {
    m_angleAdjusterMotor.set(input);
  }

  public void resetGyroAngleAdj(){
    m_angleEncoder.setPosition(0);
  }

  public double getAnglePosition(){
    return 0.0;//m_angleEncoder.getPosition();
  }

  public double getAnglePositionDegrees(){
    return 0.0;//((m_angleEncoder.getPosition() - ShooterConstants.HOOD_MOTOR_POS_AT_ZERO_ROT) * 360/ShooterConstants.HOOD_MOTOR_ROTS_PER_ROT) - 90;
  }
}
