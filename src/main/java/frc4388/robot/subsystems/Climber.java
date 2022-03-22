// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClimberConstants;
public class Climber extends SubsystemBase {
  private WPI_TalonFX elbow;

  /** Creates a new Climber */
  public Climber(WPI_TalonFX elbow) { this.elbow = elbow; }
  
  public void setEncoders(double value) { this.elbow.setSelectedSensorPosition(value); }
  public double getCurrent() { return this.elbow.getSupplyCurrent(); }
  public void setMotors(double elbowOutput) { this.elbow.set(elbowOutput * ClimberConstants.INPUT_MULTIPLIER); }

  @Override
  public void periodic() {}
}