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

  private double m_shoulderOffset;
  private double m_elbowOffset;

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

    m_shoulderOffset = m_shoulder.getSelectedSensorPosition();
    m_elbowOffset = m_elbow.getSelectedSensorPosition();

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
    if(mag >= ClimberConstants.MAX_ARM_LENGTH) {
      xTarget = (xTarget / mag) * ClimberConstants.MAX_ARM_LENGTH;
      yTarget = (yTarget / mag) * ClimberConstants.MAX_ARM_LENGTH;
      mag = ClimberConstants.MAX_ARM_LENGTH;
    } else if(mag < ClimberConstants.MIN_ARM_LENGTH && mag != 0) {
      xTarget = (xTarget / mag) * ClimberConstants.MIN_ARM_LENGTH;
      yTarget = (yTarget / mag) * ClimberConstants.MIN_ARM_LENGTH;
      mag = ClimberConstants.MIN_ARM_LENGTH;
    } else if(mag < ClimberConstants.MIN_ARM_LENGTH) {
      xTarget = ClimberConstants.MIN_ARM_LENGTH;
      yTarget = 0.d;
      mag = ClimberConstants.MIN_ARM_LENGTH;
    }
  
    // The angle to the target point
    double theta;
    if(xTarget == 0.d) {
      theta = Math.PI/2.d; // TODO rename variable
    } else {
      theta = Math.atan(yTarget / xTarget);
    }
    theta += tiltAngle;
    
    if(xTarget < 0.d)
        theta += Math.PI;
    
    
    // Correct target position for tilt
    xTarget = Math.cos(theta) * mag;
    yTarget = Math.sin(theta) * mag;
    
    // Law and Order: Cosines edition
    double shoulderAngle;
    if(mag == 0) {
      shoulderAngle = 0;
    } else {
      shoulderAngle = Math.acos((Math.pow(ClimberConstants.LOWER_ARM_LENGTH, 2) + Math.pow(mag, 2) - Math.pow(ClimberConstants.UPPER_ARM_LENGTH, 2)) /
      (2.d * ClimberConstants.LOWER_ARM_LENGTH * mag));
    }
    shoulderAngle = theta - shoulderAngle;
    
    double elbowAngle = Math.acos((Math.pow(ClimberConstants.LOWER_ARM_LENGTH, 2) + Math.pow(ClimberConstants.UPPER_ARM_LENGTH, 2) - Math.pow(mag, 2)) /
      (2.d * ClimberConstants.LOWER_ARM_LENGTH * ClimberConstants.UPPER_ARM_LENGTH));
    //elbowAngle = Math.PI - elbowAngle;
    // System.out.println("sa1: " + shoulderAngle);
    // System.out.println("ea1: " + elbowAngle);
  
    // If the climber is resting on the robot base, rotate the upper arm in the direction of the target
    if(shoulderAngle <= ClimberConstants.SHOULDER_RESTING_ANGLE) {
      shoulderAngle = ClimberConstants.SHOULDER_RESTING_ANGLE;
      double xDiff = xTarget - ClimberConstants.LOWER_ARM_LENGTH;
      // System.out.println("xDiff: " + xDiff);
  
      elbowAngle = Math.atan(-yTarget / xDiff);
      // System.out.println("ea2: " + elbowAngle);
      if(elbowAngle < 0) {
        elbowAngle = Math.PI - Math.abs(elbowAngle);
      }

      if(elbowAngle < ClimberConstants.ELBOW_RESTING_ANGLE)
        elbowAngle = ClimberConstants.ELBOW_RESTING_ANGLE;
      // System.out.println("ea3: " + elbowAngle);
  
      // Deal with negative wraparound
      // if(xDiff >= 0.d)
      //   elbowAngle += Math.PI;
      // System.out.println("ea4: " + elbowAngle);
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

    shoulderPosition += m_shoulderOffset;
    elbowPosition += m_elbowOffset;

    m_shoulder.set(TalonFXControlMode.Position, shoulderPosition);
    m_elbow.set(TalonFXControlMode.Position, elbowPosition);
  }

  public void controlWithInput(double xInput, double yInput) {
    m_position[0] += xInput * ClimberConstants.MOVE_SPEED;
    m_position[1] += yInput * ClimberConstants.MOVE_SPEED;

    System.out.println("x: " + m_position[0] + " y: " + m_position[1]);

    double tiltAngle = m_groundRelative ? getRobotTilt() : 0.d;

    // double[] testAngles = getJointAngles(0, 0, 0);
    // System.out.println("origin: " + testAngles[0] + ", " + testAngles[1]);
    
    // double[] testAngles2 = getJointAngles(5000, 5000, 0);
    // System.out.println("extended: " + testAngles2[0] + ", " + testAngles2[1]);
    
    // double[] testAngles3 = getJointAngles(0, 75, 0);
    // System.out.println("just y: " + testAngles3[0] + ", " + testAngles3[1]);
    
    // double[] testAngles4 = getJointAngles(75, 0, 0);
    // System.out.println("just x: " + testAngles4[0] + ", " + testAngles4[1]);
    
    // double[] testAngles5 = getJointAngles(-75, 0, 0);
    // System.out.println("just x: " + testAngles5[0] + ", " + testAngles5[1]);

    // double[] testAngles6 = getJointAngles(60, 25, 0);
    // System.out.println("just xy: " + testAngles6[0] + ", " + testAngles6[1]);

    double[] jointAngles = getJointAngles(m_position[0], m_position[1], tiltAngle);
    setJointAngles(jointAngles);
  }
}
