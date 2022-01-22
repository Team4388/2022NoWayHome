// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
 public CANSparkMax m_turretMotor;
  //Variables
  public Turret() {
    m_turretMotor = turretMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turnWithJoystick (double input)
  {
    m_turretMotor.set(input);

  }

  //function turnWithJoystick(double input)
  //   motor.set(input)
}



/** TODO
* setPosition function
* Limit switches
**/