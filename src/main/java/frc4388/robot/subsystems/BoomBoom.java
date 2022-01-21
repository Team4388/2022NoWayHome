// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.ShooterTables;

public class BoomBoom extends SubsystemBase {
public WPI_TalonFX m_shooterFalconLeft;
public WPI_TalonFX m_shooterFalconRight;
public ShooterTables m_shooterTable;

  /** Creates a new BoomBoom. */
  public BoomBoom(WPI_TalonFX shooterFalconLeft, WPI_TalonFX shooterFalconRight) {
  m_shooterFalconLeft = shooterFalconLeft;
  m_shooterFalconRight = shooterFalconRight;
  
    int closedLoopTimeMs = 1;
      //LEFT FALCON
      m_shooterFalconLeft.configFactoryDefault();
      m_shooterFalconLeft.setNeutralMode(NeutralMode.Coast);
      m_shooterFalconLeft.setInverted(true);
      m_shooterFalconLeft.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
      m_shooterFalconLeft.configClosedloopRamp(0.75, ShooterConstants.SHOOTER_TIMEOUT_MS);
      m_shooterFalconLeft.configPeakOutputReverse(0, ShooterConstants.SHOOTER_TIMEOUT_MS);
      m_shooterFalconLeft.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
      m_shooterFalconLeft.configClosedLoopPeriod(0, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);
      m_shooterFalconLeft.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG, ShooterConstants.SHOOTER_TIMEOUT_MS);
      
      //RIGHT FALCON
      m_shooterFalconRight.configFactoryDefault();
      m_shooterFalconRight.setNeutralMode(NeutralMode.Coast);
      m_shooterFalconRight.setInverted(false);
      m_shooterFalconRight.configOpenloopRamp(1, ShooterConstants.SHOOTER_TIMEOUT_MS);
      m_shooterFalconRight.configClosedloopRamp(0.75, ShooterConstants.SHOOTER_TIMEOUT_MS);
      //m_shooterFalconRight.configPeakOutputForward(0, ShooterConstants.SHOOTER_TIMEOUT_MS);(comment it in if necessary)
      m_shooterFalconRight.setSelectedSensorPosition(0, ShooterConstants.SHOOTER_PID_LOOP_IDX, ShooterConstants.SHOOTER_TIMEOUT_MS);
      m_shooterFalconRight.configClosedLoopPeriod(0, closedLoopTimeMs, ShooterConstants.SHOOTER_TIMEOUT_MS);
      m_shooterFalconRight.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIG, ShooterConstants.SHOOTER_TIMEOUT_MS);
      
      m_shooterTable = new ShooterTables();
  
  
  }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Abhi was here 
  }
public void runDrumShooter(double speed) {
    m_shooterFalconLeft.set(TalonFXControlMode.PercentOutput, speed);
    m_shooterFalconRight.follow(m_shooterFalconLeft);
  }    


  public void runDrumShooterVelocityPID(double targetVel) {
    m_shooterFalconLeft.set(TalonFXControlMode.Velocity, targetVel); //Init
    m_shooterFalconRight.follow(m_shooterFalconLeft);
    }
}
