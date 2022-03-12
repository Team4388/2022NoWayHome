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
  private CANSparkMax m_extenderMotor;
  private Serializer m_serializer;
  private SparkMaxLimitSwitch m_inLimit;
  private SparkMaxLimitSwitch m_outLimit;

  public boolean toggle;

  /** Creates a new Intake. */
  public Intake(WPI_TalonFX intakeMotor, CANSparkMax extenderMotor, Serializer serializer) {
    m_intakeMotor = intakeMotor;
    m_extenderMotor = extenderMotor; 
    m_serializer = serializer;

    m_extenderMotor.restoreFactoryDefaults();

    m_intakeMotor.setNeutralMode(NeutralMode.Brake);
    m_intakeMotor.setInverted(false);
    m_extenderMotor.setInverted(true);
    
    m_inLimit = m_extenderMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_outLimit = m_extenderMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_inLimit.enableLimitSwitch(true);
    m_outLimit.enableLimitSwitch(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * Runs The Intake With Triggers.
   * @param leftTrigger Left Trigger to Run -
   * @param rightTrigger Right Trigger to Run +
   */
  public void runWithTriggers(double leftTrigger, double rightTrigger) {
    m_intakeMotor.set((rightTrigger - leftTrigger) * 0.3);
    SmartDashboard.putNumber("Intake Current Supply", m_intakeMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Intake Current Stator", m_intakeMotor.getStatorCurrent());
  }
  /**
   * Runs The Extender-
   * @param extended Wether the Extender Is Extended
   */
  public void runExtender(boolean extended) {
    if (!m_serializer.getBeam() && !extended) return;
    double extenderMotorSpeed = extended ? 0.25d : -0.25d;
    m_extenderMotor.set(extenderMotorSpeed);
  }

  public void runExtender(double input) {
    if (!m_serializer.getBeam() && input < 0.) return;
    m_extenderMotor.set(input);
  }
  /**
   * Toggles The Extender
  */
  public void toggleExtender() {
    toggle = !toggle;
    runExtender(toggle);
  }

  public double getCurrent(){
    return m_intakeMotor.getSupplyCurrent() + m_extenderMotor.getOutputCurrent();
  }
}