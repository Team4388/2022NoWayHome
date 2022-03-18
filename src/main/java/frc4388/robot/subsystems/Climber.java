// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  WPI_TalonFX m_climberShoulder;
  WPI_TalonFX m_climberElbow;

  /** Creates a new Climber. */
  public Climber(WPI_TalonFX climberShoulder, WPI_TalonFX climberElbow) {
    m_climberShoulder = climberShoulder;
    m_climberElbow = climberElbow;

    m_climberShoulder.configFactoryDefault();
    m_climberElbow.configFactoryDefault();
    m_climberShoulder.setNeutralMode(NeutralMode.Brake);
    m_climberElbow.setNeutralMode(NeutralMode.Brake);
  }

  public void runShoulderWithInput(double input) {
    m_climberShoulder.set(input);
  }

  public void runElbowWithInput(double input){
    m_climberElbow.set(input);
  }

  public void runBothMotorsWithInputs(double shoulderInput, double elbowInput) {
    m_climberShoulder.set(shoulderInput);
    m_climberElbow.set(elbowInput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCurrent() {
    return m_climberElbow.getSupplyCurrent();
  }
}
