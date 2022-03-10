// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  WPI_TalonFX m_climberElbow;
  /** Creates a new Climber. */
  public Climber(WPI_TalonFX climberElbow) {
    m_climberElbow = climberElbow;
  }

  public void runWithInput(double input){
    m_climberElbow.set(input);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
