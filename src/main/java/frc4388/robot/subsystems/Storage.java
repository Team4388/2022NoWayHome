// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class Storage extends SubsystemBase {
  public CANSparkMax m_storageMotor;
  private DigitalInput m_beamShooter;
  private DigitalInput m_beamIntake;

  /** Creates a new Storage. */
  public Storage(CANSparkMax storageMotor, DigitalInput beamShooter, DigitalInput beamIntake) {
    m_storageMotor = storageMotor;
    m_beamShooter = beamShooter;
    m_beamIntake = beamIntake;
  }

  public Storage(CANSparkMax storageMotor) {
    m_storageMotor = storageMotor;
    m_beamShooter = null;
    m_beamIntake = null;
  }
  /**
   * If The Beam Is Broken, Run Storage
   * If Else, Stop Running Storage
   */
  public void manageStorage() {
    if (getBeamIntake()) runStorage(0.d);
    else runStorage(1.d);
  }
  
  /**
   * Runs The Storage at a Specifyed Speed
   * @param input The Specifyed Speed
   */
  public void runStorage(double input) {
    m_storageMotor.set(input);
  }
  /**
   * Gets The Beam State On The Shooter
   * @return The State Of The Beam on the Shooter
   */
  public boolean getBeamShooter(){
    return m_beamShooter.get();//True if open
  }

  /**
   * Gets The Beam State Of The Intake
   * @return The Beam State Of The Intake
   */
  public boolean getBeamIntake(){
    return m_beamIntake.get(); //True if open
  }
  

  @Override
  /**
   * Every Robot Tick Manage The Storage
   */
  public void periodic() {
    //manageStorage();
  }
  public double getCurrent(){
    return m_storageMotor.getOutputCurrent();
  }
}
