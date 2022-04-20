// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ExtenderConstants;

public class Extender extends SubsystemBase {
  
  private CANSparkMax m_extenderMotor;

  // private SparkMaxLimitSwitch m_inLimit;
  // private SparkMaxLimitSwitch m_outLimit;

  /** Creates a new Extender. */
  public Extender(CANSparkMax extenderMotor) {

    m_extenderMotor = extenderMotor;

    // m_inLimit = m_extenderMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    // m_outLimit = m_extenderMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    // m_inLimit.enableLimitSwitch(false);
    // m_outLimit.enableLimitSwitch(false);

    m_extenderMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ExtenderConstants.EXTENDER_FORWARD_LIMIT);
    m_extenderMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ExtenderConstants.EXTENDER_REVERSE_LIMIT);
    setExtenderSoftLimits(true);
    SmartDashboard.putData(new InstantCommand(() -> m_extenderMotor.getEncoder().setPosition(0)));
  }

  /**
   * Set status of extender motor soft limits.
   * @param set Boolean to set soft limits to.
   */
  public void setExtenderSoftLimits(boolean set) {
    m_extenderMotor.enableSoftLimit(SoftLimitDirection.kForward, set);
    m_extenderMotor.enableSoftLimit(SoftLimitDirection.kReverse, set);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extender Position", m_extenderMotor.getEncoder().getPosition());
  }

  public void runExtender(double input) {
    m_extenderMotor.set(input);
  }

  public boolean isDeployed() {
    return Math.abs(getPosition() - ExtenderConstants.EXTENDER_FORWARD_LIMIT) < ExtenderConstants.EXTENDER_TOLERANCE;
  }

  public boolean isRetracted() {
    return Math.abs(getPosition() - ExtenderConstants.EXTENDER_REVERSE_LIMIT) < ExtenderConstants.EXTENDER_TOLERANCE;
  }

  public double getPosition() {
    return m_extenderMotor.getEncoder().getPosition();
  }

  public void setEncoder(double position) {
    m_extenderMotor.getEncoder().setPosition(position);
  }

  public double getCurrent() {
    return m_extenderMotor.getOutputCurrent();
  }
}
