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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;
import frc4388.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  public WPI_TalonFX m_shoulder;
  public WPI_TalonFX m_elbow;

  private double m_shoulderOffset;
  private double m_elbowOffset;

  private WPI_PigeonIMU m_gyro;
  private boolean m_groundRelative;
  private double m_robotAngle;
  private double m_robotPosition;
  
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

    m_elbow.configForwardSoftLimitThreshold(ClimberConstants.ELBOW_SOFT_LIMIT_FORWARD);
    m_elbow.configForwardSoftLimitEnable(false);
    m_elbow.configReverseSoftLimitThreshold(ClimberConstants.ELBOW_SOFT_LIMIT_REVERSE);
    m_elbow.configReverseSoftLimitEnable(false);

    m_shoulder.configForwardSoftLimitThreshold(ClimberConstants.SHOULDER_SOFT_LIMIT_FORWARD);
    m_shoulder.configForwardSoftLimitEnable(false);
    m_shoulder.configReverseSoftLimitThreshold(ClimberConstants.SHOULDER_SOFT_LIMIT_REVERSE);
    m_shoulder.configReverseSoftLimitEnable(false);

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

public void setMotors(double shoulderOutput, double elbowOutput) {
  m_shoulder.set(shoulderOutput);
  m_elbow.set(elbowOutput);
}

/* Rotation Matrix
 R = [cos(0) -sin(0) \n sin(0) cos(0)] 
 Rv = [cos(0) -sin(0) \n sin(0) cos(0)] = [x \n y] = [xcos(0) - ysin(0), xsin(0) + ycos(0)]
 Rotation Matrix resource: https://en.wikipedia.org/wiki/Rotation_matrix 
 Rotation Matrix video: https://youtu.be/Ta8cKqltPfU 
 */
  public double getRobotTilt() {
    double[] ypr = new double[3];
    m_gyro.getYawPitchRoll(ypr);
    double theta = 0;
    
    //  double sin;
    //  double cos;
    //  xsin = 0; //placeholder for sin, cos
    //  ysin = 0;
    //  xcos = 0;
    //  ycos = 0;
    double[][] rotMax = {
      {Math.cos(theta) - Math.sin(theta), 0 },
      {Math.sin(theta) + Math.cos(theta), 0},
      {0, 0, 1}
    };

    if (m_robotPosition != m_robotAngle){
      setRobotAngle(ClimberConstants.ROBOT_ANGLE_ID, rotMax, m_robotPosition);
    }

    return Math.toRadians(ypr[1]); // Pitch
    // multiply by pie and then divide by 180
    
  }

  public void setClimberGains() {
    // shoulder PIDs
    m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
    m_shoulder.config_kF(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kP(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kI(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kD(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kD, ClimberConstants.CLIMBER_TIMEOUT_MS);

    // elbow PIDs
    m_elbow.selectProfileSlot(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
    m_elbow.config_kF(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kP(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kI(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kD(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kD, ClimberConstants.CLIMBER_TIMEOUT_MS);
  }

  public void setJointAngles(double[] angles) {
    System.out.println(angles);
    setJointAngles(angles[0], angles[1]);
  }

  public void setJointAngles(double shoulderAngle, double elbowAngle) {
    shoulderAngle = shoulderAngle < ClimberConstants.SHOULDER_RESTING_ANGLE ? ClimberConstants.SHOULDER_RESTING_ANGLE : shoulderAngle;
    elbowAngle = elbowAngle < ClimberConstants.ELBOW_RESTING_ANGLE ? ClimberConstants.ELBOW_RESTING_ANGLE : elbowAngle;

    shoulderAngle = shoulderAngle > ClimberConstants.SHOULDER_MAX_ANGLE ? ClimberConstants.SHOULDER_MAX_ANGLE : shoulderAngle;
    elbowAngle = elbowAngle > ClimberConstants.ELBOW_MAX_ANGLE ? ClimberConstants.ELBOW_MAX_ANGLE : elbowAngle;

    // Convert radians to ticks
    System.out.println("angles: " + shoulderAngle + ", " + elbowAngle);

    double shoulderPosition = (shoulderAngle * (Constants.TICKS_PER_ROTATION_FX/2.d)) / Math.PI;
    double elbowPosition = (elbowAngle * (Constants.TICKS_PER_ROTATION_FX/2.d)) / Math.PI;

    shoulderAngle *= ClimberConstants.GEAR_BOX_RATIO;
    elbowAngle *= ClimberConstants.GEAR_BOX_RATIO;

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
  
  public void setRobotAngle(double robotAngle, double[][] rotMax, double robotPosition) {
    m_robotPosition = robotPosition;
    m_robotAngle = robotAngle;
    m_robotAngle = 45; //45 is placeholder
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Shoulder", m_shoulder.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbow", m_elbow.getSelectedSensorPosition());
  }
  
}
