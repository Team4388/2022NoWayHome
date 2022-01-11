// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.mocks;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;

/**
 * Add your docs here.
 */
public class MockPigeonIMU extends PigeonIMU {
  public int m_deviceNumber;
  public double currentYaw;
  public double currentPitch;
  public double currentRoll;

  public MockPigeonIMU(int deviceNumber) {
    super(deviceNumber);
    m_deviceNumber = deviceNumber;
  }

  @Override
  public ErrorCode setYaw(double angleDeg) {
    currentYaw = angleDeg;
    return ErrorCode.OK;
  }

  /**
   * @param currentPitch the Pitch to set
   */
  public void setCurrentPitch(double currentPitch) {
    this.currentPitch = currentPitch;
  }

  /**
   * @param currentRoll the Roll to set
   */
  public void setCurrentRoll(double currentRoll) {
    this.currentRoll = currentRoll;
  }

  @Override
  public ErrorCode getYawPitchRoll(double[] ypr_deg) {
    ypr_deg[0] = currentYaw;
    ypr_deg[1] = currentPitch;
    ypr_deg[2] = currentRoll;
    return ErrorCode.OK;
  }
}
