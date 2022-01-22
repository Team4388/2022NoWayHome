// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
Safety
Hooks
Add
*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;
import frc4388.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private WPI_TalonFX m_shoulder;
  private WPI_TalonFX m_elbow;

  private WPI_PigeonIMU m_gyro;
  private boolean m_groundRelative;

  private double[] m_position = {ClimberConstants.MIN_ARM_LENGTH, 0.d};
  
  public Climber(WPI_TalonFX shoulder, WPI_TalonFX elbow, WPI_PigeonIMU gyro, boolean groundRelative) {
    m_shoulder = shoulder;
    m_shoulder.configFactoryDefault();
    m_shoulder.setNeutralMode(NeutralMode.Brake);

    m_elbow = elbow;
    m_elbow.configFactoryDefault();
    m_elbow.setNeutralMode(NeutralMode.Brake);

    setClimberGains();

    if(groundRelative)
      m_gyro = gyro;
    
    m_groundRelative = groundRelative;
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
      mag = ClimberConstants.MAX_ARM_LENGTH;
    } else if(mag < ClimberConstants.MIN_ARM_LENGTH) {
      xTarget = (xTarget / mag) * ClimberConstants.MIN_ARM_LENGTH;
      yTarget = (yTarget / mag) * ClimberConstants.MIN_ARM_LENGTH;
      mag = ClimberConstants.MIN_ARM_LENGTH;
    }

    // The angle to the target point
    double theta = Math.atan(yTarget / xTarget) + tiltAngle; // TODO rename variable
    if(xTarget < 0.d)
      theta += Math.PI;
    
    // Correct target position for tilt
    xTarget = Math.cos(theta) * mag;
    yTarget = Math.sin(theta) * mag;
    
    // Law and Order: Cosines edition
    double shoulderAngle = Math.acos((Math.pow(ClimberConstants.LOWER_ARM_LENGTH, 2) + Math.pow(mag, 2) - Math.pow(ClimberConstants.UPPER_ARM_LENGTH, 2)) /
      (2.d * ClimberConstants.LOWER_ARM_LENGTH * mag));
      //shoulderAngle = acos(LowerArmLength^2 + mag^2)
    shoulderAngle = theta - shoulderAngle;
    
    double elbowAngle = Math.acos((Math.pow(ClimberConstants.LOWER_ARM_LENGTH, 2) + Math.pow(ClimberConstants.UPPER_ARM_LENGTH, 2) - Math.pow(mag, 2)) /
      (2.d * ClimberConstants.LOWER_ARM_LENGTH * ClimberConstants.UPPER_ARM_LENGTH));
    elbowAngle = Math.PI - elbowAngle;

    // If the climber is resting on the robot base, rotate the upper arm in the direction of the target
    if(shoulderAngle <= ClimberConstants.SHOULDER_RESTING_ANGLE) {
      shoulderAngle = ClimberConstants.SHOULDER_RESTING_ANGLE;
      double xDiff = xTarget - ClimberConstants.LOWER_ARM_LENGTH;

      elbowAngle = Math.atan(-yTarget / xDiff);
      if(elbowAngle < ClimberConstants.ELBOW_RESTING_ANGLE)
        elbowAngle = ClimberConstants.ELBOW_RESTING_ANGLE;


      elbowAngle = Math.PI - elbowAngle;

      // Deal with negative wraparound
      if(xDiff >= 0.d)
        elbowAngle += Math.PI;
    }

    angles[0] = shoulderAngle;
    angles[1] = elbowAngle;
    return angles;
  }

  public double getRobotTilt() {
    double[] ypr = new double[3];
    m_gyro.getYawPitchRoll(ypr);
    return Math.toRadians(ypr[1]); // Pitch
  }

  public void setClimberGains() {
    // shoulder PIDs
    m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
    m_shoulder.config_kF(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.m_kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kP(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.m_kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kI(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.m_kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kD(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.m_kD, ClimberConstants.CLIMBER_TIMEOUT_MS);

    // elbow PIDs
    m_elbow.selectProfileSlot(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
    m_elbow.config_kF(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.m_kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kP(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.m_kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kI(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.m_kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kD(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.m_kD, ClimberConstants.CLIMBER_TIMEOUT_MS);
  }

  public void setJointAngles(double[] angles) {
    System.out.println(angles);
    setJointAngles(angles[0], angles[1]);
  }

  public void setJointAngles(double shoulderAngle, double elbowAngle) {
    // Convert radians to ticks
    System.out.println("angles: " + shoulderAngle + ", " + elbowAngle);

    double shoulderPosition = (shoulderAngle * (Constants.TICKS_PER_ROTATION_FX/2.d)) / Math.PI;
    double elbowPosition = (elbowAngle * (Constants.TICKS_PER_ROTATION_FX/2.d)) / Math.PI;

    m_shoulder.set(TalonFXControlMode.Position, shoulderPosition);
    m_elbow.set(TalonFXControlMode.Position, elbowPosition);
  }

  public void controlWithInput(double xInput, double yInput) {
    m_position[0] += xInput * ClimberConstants.MOVE_SPEED;
    m_position[1] += yInput * ClimberConstants.MOVE_SPEED;

    System.out.println("x: " + m_position[0] + " y: " + m_position[1]);

    double tiltAngle = m_groundRelative ? getRobotTilt() : 0.d;

    System.out.println(getJointAngles(0, 0, 0));

    double[] jointAngles = getJointAngles(m_position[0], m_position[1], tiltAngle);
    setJointAngles(jointAngles);
  }
}
