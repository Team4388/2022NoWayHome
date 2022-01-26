// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.Base64.Encoder;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.ShooterTables;
import frc4388.utility.Gains;
import frc4388.utility.controller.IHandController;

public class BoomBoom extends SubsystemBase {
public WPI_TalonFX m_shooterFalconLeft = new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_BALLER_ID);
public WPI_TalonFX m_shooterFalconRight= new WPI_TalonFX(ShooterConstants.SHOOTER_FALCON_BALLER_FOLLOWER_ID);
public ShooterTables m_shooterTable;
public static Gains m_drumShooterGains = ShooterConstants.DRUM_SHOOTER_GAINS;
public static BoomBoom m_boomBoom;
public static IHandController m_driverController; //not sure if driverController in 2022 = m_controller in 2020

double velP;
double input;

public boolean m_isDrumReady = false;
public double m_fireVel;



public Hood m_hoodSubsystem;
public Turret m_turretSubsystem;

/*
* Creates new BoomBoom subsystem, has drum shooter and angle adjuster
*/
public BoomBoom(){
//Testing purposes resetting gyros
//resetGryoAngleADj();

}
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
try {
  // SmartDashboard.putNumber("Drum Velocity", m_shooterFalconLeft.getSelectedSensorVelocity());

  // SmartDashboard.putNumber("Drum Velocity CSV", m_fireVel);

  // SmartDashboard.putNumber("Shooter Temp C", m_shooterFalconLeft.getTemperature()); //all these values should be "defined" elsewhere, fix this

  // SmartDashboard.putNumber("Shooter Current", m_shooterFalconLeft.getSupplyCurrent());

  // SmartDashboard.putNumber("Drum Ready", m_isDrumReady);
} catch (Exception e) {
  //TODO: handle exception
}

  }

  public void passRequiredSubsystem(Hood subsystem0, Turret subsystem1) {
    m_hoodSubsystem = subsystem0;
    m_turretSubsystem = subsystem1;
  }

  public double addFireVel(){
    return m_fireVel;
  }
public void runDrumShooter(double speed) {
    m_shooterFalconLeft.set(TalonFXControlMode.PercentOutput, speed);
    m_shooterFalconRight.follow(m_shooterFalconLeft);
  }    

public void setShooterGains() {
  m_shooterFalconLeft.selectProfileSlot(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.SHOOTER_PID_LOOP_IDX);
  m_shooterFalconLeft.config_kF(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kF, ShooterConstants.SHOOTER_TIMEOUT_MS);
  m_shooterFalconLeft.config_kP(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kP, ShooterConstants.SHOOTER_TIMEOUT_MS);
  m_shooterFalconLeft.config_kI(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kI, ShooterConstants.SHOOTER_TIMEOUT_MS);
  m_shooterFalconLeft.config_kD(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kD, ShooterConstants.SHOOTER_TIMEOUT_MS);
}

  public void runDrumShooterVelocityPID(double targetVel) {
    m_shooterFalconLeft.set(TalonFXControlMode.Velocity, targetVel); //Init
    m_shooterFalconRight.follow(m_shooterFalconLeft);
    // New BoomBoom controller stuff 
    BangBangController controller = new BangBangController();
    //Controls a motor with the output of the BangBang controller
    //Controls a motor with the output of the BangBang conroller and a feedforward
    //Shrinks the feedforward slightly to avoid over speeding the shooter
    m_shooterFalconLeft.set(controller.calculate(Encoder.getRate(), targetVel) + 0.9 * feedforward.calculate(targetVel);
  
  
  }
}
