// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

//Imported Limit switch ONLY
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class Intake extends SubsystemBase {

  private WPI_TalonFX m_intakeMotor;
  private CANSparkMax m_extenderMotor;
  private SparkMaxLimitSwitch m_inLimit;
  private SparkMaxLimitSwitch m_outLimit;

  public boolean toggle;

  /** Creates a new Intake. */
  public Intake(WPI_TalonFX intakeMotor, CANSparkMax extenderMotor) {
    m_intakeMotor = intakeMotor;
    m_extenderMotor = extenderMotor; 

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
    m_intakeMotor.set(rightTrigger - leftTrigger);
  }
  /**
   * Runs The Extender
   * @param extended Wether the Extender Is Extended
   */
  public void runExtender(boolean extended) { //TODO: Do not bring intake in if there is a ball in the extender (check if intake being in brakes the beam brake?)
    double extenderMotorSpeed = extended ? 0.25d : 0.d;
    m_extenderMotor.set(extenderMotorSpeed);
  }
  /**
   * Toggles The Extender
  */
  public void toggleExtender() {
    toggle = !toggle;
    runExtender(toggle);
  }
  //Test
}