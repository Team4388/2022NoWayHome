// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc4388.robot.Constants.StorageConstants;

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
  /**
   * If The Beam Is Broken, Run Storage
   * If Else, Stop Running Storage
   */
  public void manageStorage() {
    if (getBeamIntake()) { //Maybe needs to be shooter
      runStorage(1.d);
    } else { runStorage(0.d); }
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
    return m_beamShooter.get();
  }

  /**
   * Gets The Beam State Of The Intake
   * @return The Beam State Of The Intake
   */
  public boolean getBeamIntake(){
    return m_beamIntake.get();
  }
  

  @Override
  /**
   * Every Robot Tick Manage The Storage
   */
  public void periodic() {
    manageStorage();
  }
}
