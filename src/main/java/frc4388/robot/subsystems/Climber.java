// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private WPI_TalonFX m_shoulder;
  private WPI_TalonFX m_elbow;

  private WPI_PigeonIMU m_gyro;
  
  public Climber(WPI_TalonFX shoulder, WPI_TalonFX elbow, WPI_PigeonIMU gyro) {
    m_shoulder = shoulder;
    m_elbow = elbow;
    m_gyro = gyro;
  }

  /* getJointAngles
   * Gets joint angles for climber arm
   * xTarget and yTarget are set in the xy plane of the climber, accounting for the
   * rotation of the robot. Both parameters should be in cm.
   * returns [shoulderAngle, elbowAngle] in radians
   * Does not set the motors automatically
   * 
   * IK resource: https://devforum.roblox.com/t/2-joint-2-limb-inverse-kinematics/252399 */
  public static double[] getJointAngles(double xTarget, double yTarget, double tiltAngle) {
    double [] angles = new double[2];

    double mag = Math.hypot(xTarget, yTarget);
    if(mag > ClimberConstants.MAX_ARM_LENGTH) {
      xTarget = (xTarget / mag) * ClimberConstants.MAX_ARM_LENGTH;
      yTarget = (yTarget / mag) * ClimberConstants.MAX_ARM_LENGTH;
    } else if(mag < ClimberConstants.MIN_ARM_LENGTH) {
      xTarget = (xTarget / mag) * ClimberConstants.MIN_ARM_LENGTH;
      yTarget = (yTarget / mag) * ClimberConstants.MIN_ARM_LENGTH;
    }

    // The angle to the target point
    double theta = Math.atan(yTarget / xTarget) + tiltAngle; // TODO rename variable
    // Correct target position for tilt
    xTarget = Math.cos(theta) * mag;
    yTarget = Math.sin(theta) * mag;
    
    // Law and Order: Cosines edition
    double shoulderAngle = Math.acos((Math.pow(ClimberConstants.LOWER_ARM_LENGTH, 2) + Math.pow(mag, 2) -Math.pow(ClimberConstants.UPPER_ARM_LENGTH, 2)) /
      (2.d * ClimberConstants.LOWER_ARM_LENGTH * mag));
    shoulderAngle = theta - shoulderAngle;
    
    double elbowAngle = Math.acos((Math.pow(ClimberConstants.LOWER_ARM_LENGTH, 2) + Math.pow(ClimberConstants.UPPER_ARM_LENGTH, 2) - Math.pow(mag, 2)) /
      (2.d * ClimberConstants.LOWER_ARM_LENGTH * ClimberConstants.UPPER_ARM_LENGTH));
    elbowAngle = Math.PI - elbowAngle;

    // If the climber is resting on the robot base, rotate the upper arm in the direction of the target
    if(shoulderAngle <= 0.d) {
      shoulderAngle = 0.d;
      double xDiff = xTarget - ClimberConstants.LOWER_ARM_LENGTH;

      elbowAngle = Math.atan(-yTarget / xDiff);
      elbowAngle = Math.PI - elbowAngle;

      if(xDiff >= 0.d) {
        elbowAngle += Math.PI;
      }
    }

    return angles;
  }

  public void setJointAngles(double[] angles) {
    setJointAngles(angles[0], angles[1]);
  }

  public void setJointAngles(double shoulderAngle, double elbowAngle) {
    // Set PIDs
  }
}
