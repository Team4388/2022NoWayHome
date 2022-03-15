// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClimberConstants;

public class ClimberRewrite extends SubsystemBase {
  private static HashMap<Double, Double> shoulderFeedMap;
  private static HashMap<Double, Double> elbowFeedMap;

  public static void configureFeedMaps() {
    
  }

  private WPI_TalonFX m_shoulder;
  private WPI_TalonFX m_elbow;
  private WPI_Pigeon2 m_gyro;

  private double shoulderStartPosition;
  private double elbowStartPosition;

  private boolean groundRelative;

  /** Creates a new ClimberRewrite. */
  public ClimberRewrite(WPI_TalonFX shoulder, WPI_TalonFX elbow, WPI_Pigeon2 gyro, boolean groundRelative_) {
    m_shoulder = shoulder;
    m_shoulder.configFactoryDefault();
    m_shoulder.setNeutralMode(NeutralMode.Brake);

    m_elbow = elbow;
    m_elbow.configFactoryDefault();
    m_elbow.setNeutralMode(NeutralMode.Brake);

    setClimberGains();

    shoulderStartPosition = m_shoulder.getSelectedSensorPosition();
    elbowStartPosition = m_elbow.getSelectedSensorPosition();

    m_elbow.configForwardSoftLimitThreshold(ClimberConstants.ELBOW_SOFT_LIMIT_FORWARD);
    m_elbow.configForwardSoftLimitEnable(false);
    m_elbow.configReverseSoftLimitThreshold(ClimberConstants.ELBOW_SOFT_LIMIT_REVERSE);
    m_elbow.configReverseSoftLimitEnable(false);

    m_shoulder.configForwardSoftLimitThreshold(ClimberConstants.SHOULDER_SOFT_LIMIT_FORWARD);
    m_shoulder.configForwardSoftLimitEnable(false);
    m_shoulder.configReverseSoftLimitThreshold(ClimberConstants.SHOULDER_SOFT_LIMIT_REVERSE);
    m_shoulder.configReverseSoftLimitEnable(false);

    if(groundRelative)
      m_gyro = gyro;
    
    groundRelative = groundRelative_;
  }

  public void setClimberGains() {
    m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
    m_shoulder.config_kF(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kP(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kI(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kD(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kD, ClimberConstants.CLIMBER_TIMEOUT_MS);

    m_elbow.selectProfileSlot(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
    m_elbow.config_kF(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kP(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kI(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kD(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kD, ClimberConstants.CLIMBER_TIMEOUT_MS);
  }

  public void setClimberFeedForward(double shoulderF, double elbowF) {
    m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
    m_shoulder.config_kF(ClimberConstants.SHOULDER_SLOT_IDX, shoulderF, ClimberConstants.CLIMBER_TIMEOUT_MS);

    m_elbow.selectProfileSlot(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
    m_elbow.config_kF(ClimberConstants.ELBOW_SLOT_IDX, elbowF, ClimberConstants.CLIMBER_TIMEOUT_MS);
  }

  public void compensateFeedForArmAngles() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
