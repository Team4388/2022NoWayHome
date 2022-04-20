// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private WPI_TalonFX m_intakeMotor;

  /** Creates a new Intake. */
  public Intake(WPI_TalonFX intakeMotor) {
    m_intakeMotor = intakeMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Percent Output", m_intakeMotor.get());
  }
  /**
   * Runs The Intake With Triggers as input
   * @param leftTrigger Left Trigger to Run Inward
   * @param rightTrigger Right Trigger to Run Outward
   */
  public void runWithTriggers(double leftTrigger, double rightTrigger) {
    m_intakeMotor.set((rightTrigger - leftTrigger) * IntakeConstants.INTAKE_SPEED_MULTIPLIER);
    SmartDashboard.putNumber("Intake Current Supply", m_intakeMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Intake Current Stator", m_intakeMotor.getStatorCurrent());
  }

  public void runAtOutput(double output, double multiplier) {
    m_intakeMotor.set(output * multiplier);
  }

  public void runAtOutput(double output) {
    m_intakeMotor.set(output * IntakeConstants.INTAKE_SPEED_MULTIPLIER);
  }
  
  public double getCurrent() {
    return m_intakeMotor.getSupplyCurrent();
  }
}