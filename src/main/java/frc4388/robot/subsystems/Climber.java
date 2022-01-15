// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  WPI_TalonFX m_shoulder;
  WPI_TalonFX m_elbow;
  
  public Climber(WPI_TalonFX shoulder, WPI_TalonFX elbow) {
    m_shoulder = shoulder;
    m_elbow = elbow;
  }

  /* getJointAngles
   * Gets joint angles for climber arm
   * xTarget and yTarget are set in the xy plane of the climber, not accounting for the
   * rotation of the robot. Both parameters should be in cm.
   * returns [shoulderAngle, elbowAngle]
   * Does not set the motors automatically
   * 
   * IK resource: https://devforum.roblox.com/t/2-joint-2-limb-inverse-kinematics/252399 */
  public double[] getJointAngles(double xTarget, double yTarget) {
    double mag = Math.hypot(xTarget, yTarget);
    double upperArm_2 = ClimberConstants.UPPER_ARM_LENGTH * ClimberConstants.UPPER_ARM_LENGTH;
    double lowerArm_2 = ClimberConstants.LOWER_ARM_LENGTH * ClimberConstants.LOWER_ARM_LENGTH;

    double shoulderAngle = Math.acos((-lowerArm_2 + upperArm_2 - mag) / (2.d * ClimberConstants.UPPER_ARM_LENGTH * mag));
    double elbowAngle = Math.acos((lowerArm_2 + upperArm_2 - mag) / (2.d * ClimberConstants.LOWER_ARM_LENGTH * ClimberConstants.UPPER_ARM_LENGTH));

    return null;
  }
}
