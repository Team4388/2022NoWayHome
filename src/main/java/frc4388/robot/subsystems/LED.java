// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc4388.robot.Constants.LEDConstants;
import frc4388.utility.LEDPatterns;

/**
 * Allows for the control of a 5v LED Strip using a Rev Robotics Blinkin LED
 * Driver
 */
public class LED extends SubsystemBase {

  private LEDPatterns m_currentPattern;
  private Spark m_LEDController;

  /**
   * Add your docs here.
   */
  public LED(Spark LEDController){
    m_LEDController = LEDController;
    setPattern(LEDConstants.DEFAULT_PATTERN);
    updateLED();
    System.err.println("In the Beginning, there was Joe.\nAnd he said, 'Let there be LEDs.'\nAnd it was good.");
  }

  @Override
  public void periodic(){
    //SmartDashboard.putNumber("LED", m_currentPattern.getValue());
  }

  /**
   * Add your docs here.
   */
  public void updateLED(){
    m_LEDController.set(m_currentPattern.getValue());
  }

  /**
   * Add your docs here.
   */
  public void setPattern(LEDPatterns pattern){
    m_currentPattern = pattern;
    m_LEDController.set(m_currentPattern.getValue());
  }

  /**
   * Add your docs here.
   * @return
   */
  public LEDPatterns getPattern() {
    return m_currentPattern;
  }
}