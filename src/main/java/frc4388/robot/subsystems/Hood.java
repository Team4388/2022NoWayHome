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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.Gains;

public class Hood extends SubsystemBase {
  // public BoomBoom m_shooterSubsystem;
  
  public SparkMaxLimitSwitch m_hoodUpLimitSwitch;
  public SparkMaxLimitSwitch m_hoodDownLimitSwitch;
  // public static Gains m_angleAdjusterGains = ShooterConstants.SHOOTER_ANGLE_GAINS;
  public RelativeEncoder m_angleEncoder;
  public CANSparkMax m_angleAdjustMotor;
  public SparkMaxPIDController m_angleAdjusterPIDController;
  
  
  public boolean m_isHoodReady = false;

public double m_fireAngle;
  

  /** Creates a new Hood. */
  public Hood(CANSparkMax angleAdjustMotor) {
    m_angleAdjustMotor = angleAdjustMotor;
     m_angleAdjustMotor.setIdleMode(IdleMode.kBrake);
     m_angleEncoder= m_angleAdjustMotor.getEncoder();
     m_angleAdjusterPIDController = m_angleAdjustMotor.getPIDController();
    m_hoodUpLimitSwitch = m_angleAdjustMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_hoodDownLimitSwitch = m_angleAdjustMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_hoodUpLimitSwitch.enableLimitSwitch(true);
    m_hoodDownLimitSwitch.enableLimitSwitch(true);
  }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  // public void runAngleAdjustPID(double targetAngle) 
  // {
  //   //Set PID Coefficients
  //   m_angleAdjusterPIDController.setP(m_angleAdjusterGains.m_kP);
  //   m_angleAdjusterPIDController.setI(m_angleAdjusterGains.m_kI);
  //   m_angleAdjusterPIDController.setD(m_angleAdjusterGains.m_kD);
  //   m_angleAdjusterPIDController.setIZone(m_angleAdjusterGains.m_kIzone);
  //   m_angleAdjusterPIDController.setFF(m_angleAdjusterGains.m_kF);
  //   m_angleAdjusterPIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN / 5, m_angleAdjusterGains.m_kPeakOutput / 5);

  //   m_angleAdjusterPIDController.setReference(targetAngle, ControlType.kPosition);
  // }
  

  public void runHood(double input) {
    input *= .6;
    m_angleAdjustMotor.set(input);
  }



  public void resetGyroAngleAdj(){
    m_angleEncoder.setPosition(0);
  }

  public double getAnglePositionPID() {
    return m_angleEncoder.getPosition();
  }

  // public double getAnglePositionDegrees(){
    // return ((m_angleEncoder.getPosition() - ShooterConstants.HOOD_MOTOR_POS_AT_ZERO_ROT) * 360/ShooterConstants.HOOD_MOTOR_ROTS_PER_ROT) - 90;
  // }
}
