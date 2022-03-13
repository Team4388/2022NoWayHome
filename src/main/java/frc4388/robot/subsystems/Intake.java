// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

//Imported Limit switch ONLY
import com.revrobotics.SparkMaxLimitSwitch;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

public class Intake extends SubsystemBase {

  private WPI_TalonFX m_intakeMotor;
  private Serializer m_serializer;

  /** Creates a new Intake. */
  public Intake(WPI_TalonFX intakeMotor, Serializer serializer) {
    m_intakeMotor = intakeMotor;
    m_serializer = serializer;

    m_intakeMotor.setNeutralMode(NeutralMode.Brake);
    m_intakeMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * Runs The Intake With Triggers as input
   * @param leftTrigger Left Trigger to Run Inward
   * @param rightTrigger Right Trigger to Run Outward
   */
  public void runWithTriggers(double leftTrigger, double rightTrigger) {
    m_intakeMotor.set((rightTrigger - leftTrigger) * 0.3);
    SmartDashboard.putNumber("Intake Current Supply", m_intakeMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Intake Current Stator", m_intakeMotor.getStatorCurrent());
  }
  
  public double getCurrent() {
    return m_intakeMotor.getSupplyCurrent();
  }
}