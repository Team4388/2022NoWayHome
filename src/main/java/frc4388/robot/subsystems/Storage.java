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
  public CANSparkMax m_storageMotor = new CANSparkMax(StorageConstants.STORAGE_CAN_ID, MotorType.kBrushless);
  private DigitalInput m_beamShooter = new DigitalInput(StorageConstants.BEAM_SENSOR_SHOOTER);
  private DigitalInput m_beamIntake = new DigitalInput(StorageConstants.BEAM_SENSOR_INTAKE);

  /** Creates a new Storage. */
  public Storage() {

  }
  public void manageStorage() {
    if (m_beamShooter.get()) {
      runStorage(1.d);
    } else { runStorage(0.d); }
  }
  public void runStorage(double input) {
    m_storageMotor.set(input);
  }

  public boolean getBeamShooter(){
    return m_beamShooter.get();
  }

  public boolean getBeamIntake(){
    return m_beamIntake.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
