// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {
  
  CANSparkMax m_extenderMotor;

  private SparkMaxLimitSwitch m_inLimit;
  private SparkMaxLimitSwitch m_outLimit;

  public boolean toggle;

  /** Creates a new Extender. */
  public Extender(CANSparkMax extenderMotor) {

    m_extenderMotor = extenderMotor;

    m_extenderMotor.restoreFactoryDefaults();
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
   * Runs The Extender-
   * @param extended Wether the Extender Is Extended
   */
  // public void runExtender(boolean extended) {
  //   if (!m_serializer.getBeam() && !extended) return;
  //   double extenderMotorSpeed = extended ? 0.25d : -0.25d;
  //   m_extenderMotor.set(extenderMotorSpeed);
  // }

  public void runExtender(double input) {
    // if (!m_serializer.getBeam() && input < 0.) return;
    m_extenderMotor.set(input);
  }

  public double getCurrent() {
    return m_extenderMotor.getOutputCurrent();
  }

  /**
   * Toggles The Extender
  */
  // public void toggleExtender() {
  //   toggle = !toggle;
  //   runExtender(toggle);
  // }
}
