// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private WPI_TalonFX m_intakeMotor;
  private WPI_TalonFX m_extenderMotor;

  /** Creates a new Intake. */
  public Intake(WPI_TalonFX intakeMotor, WPI_TalonFX extenderMotor) {
    m_intakeMotor = intakeMotor;
    m_extenderMotor = extenderMotor; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runWithTriggers(double leftTrigger, double rightTrigger) {
    m_intakeMotor.set(rightTrigger - leftTrigger);
  }

  public void runExtender(boolean extended) {
    m_extenderMotor.set(extended ? 1 : -1);
  }
}

/*
 Function toggle extender
            bool = !bool
Configure limit switches forward and reverse
*/