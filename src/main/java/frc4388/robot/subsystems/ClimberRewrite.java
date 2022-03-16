// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;
import frc4388.robot.Constants.ClimberConstants;

public class ClimberRewrite extends SubsystemBase {
  private static double[][] shoulderFeedMap;
  private static double[][] elbowFeedMap;

  public static void configureFeedMaps() {
    
  }

  private WPI_TalonFX m_shoulder;
  private WPI_TalonFX m_elbow;
  private WPI_Pigeon2 m_gyro;

  private Point tPoint;

  private boolean groundRelative;

  /** Creates a new ClimberRewrite. */
  public ClimberRewrite(WPI_TalonFX shoulder, WPI_TalonFX elbow, WPI_Pigeon2 gyro, boolean _groundRelative) {
    m_shoulder = shoulder;
    m_shoulder.configFactoryDefault();
    m_shoulder.setNeutralMode(NeutralMode.Brake);

    m_elbow = elbow;
    m_elbow.configFactoryDefault();
    m_elbow.setNeutralMode(NeutralMode.Brake);

    setClimberGains();

    // shoulderStartPosition = m_shoulder.getSelectedSensorPosition();
    // elbowStartPosition = m_elbow.getSelectedSensorPosition();
    m_shoulder.setSelectedSensorPosition(((ClimberConstants.SHOULDER_RESTING_ANGLE * (Constants.TICKS_PER_ROTATION_FX/2.d)) / Math.PI) * ClimberConstants.SHOULDER_GB_RATIO);
    m_elbow.setSelectedSensorPosition(((ClimberConstants.ELBOW_RESTING_ANGLE * (Constants.TICKS_PER_ROTATION_FX/2.d)) / Math.PI) * ClimberConstants.SHOULDER_GB_RATIO);

    m_elbow.configForwardSoftLimitThreshold(ClimberConstants.ELBOW_SOFT_LIMIT_FORWARD);
    m_elbow.configForwardSoftLimitEnable(false);
    m_elbow.configReverseSoftLimitThreshold(ClimberConstants.ELBOW_SOFT_LIMIT_REVERSE);
    m_elbow.configReverseSoftLimitEnable(false);

    m_shoulder.configForwardSoftLimitThreshold(ClimberConstants.SHOULDER_SOFT_LIMIT_FORWARD);
    m_shoulder.configForwardSoftLimitEnable(false);
    m_shoulder.configReverseSoftLimitThreshold(ClimberConstants.SHOULDER_SOFT_LIMIT_REVERSE);
    m_shoulder.configReverseSoftLimitEnable(false);

    tPoint = new Point();

    if(groundRelative)
      m_gyro = gyro;
    
    groundRelative = _groundRelative;
  }

  public void setClimberGains() {
    m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
    m_shoulder.config_kF(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kP(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kI(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kD(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_GAINS.kD, ClimberConstants.CLIMBER_TIMEOUT_MS);

    m_elbow.selectProfileSlot(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
    m_elbow.config_kF(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kP(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kI(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kD(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_GAINS.kD, ClimberConstants.CLIMBER_TIMEOUT_MS);
  }

  public void setClimberFeedForward(double shoulderF, double elbowF) {
    m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
    m_shoulder.config_kF(ClimberConstants.SHOULDER_SLOT_IDX, shoulderF, ClimberConstants.CLIMBER_TIMEOUT_MS);

    m_elbow.selectProfileSlot(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
    m_elbow.config_kF(ClimberConstants.ELBOW_SLOT_IDX, elbowF, ClimberConstants.CLIMBER_TIMEOUT_MS);
  }

  public void compensateFeedForArmAngles() {
    double shoulderPosition = m_shoulder.getSelectedSensorPosition();
    double elbowPosition = m_elbow.getSelectedSensorPosition();

    double shoulderFeed = 0;
    double elbowFeed = 0;

    for(int i = 1; i < shoulderFeedMap.length; i++) {
      if(shoulderFeedMap[i][0] > shoulderPosition) {
        double percentDifference = (shoulderPosition - shoulderFeedMap[i-1][0]) / (shoulderFeedMap[i][0] - shoulderFeedMap[i-1][0]);
        double feedDifference = shoulderFeedMap[i][1] - shoulderFeedMap[i-1][1];
        shoulderFeed = (percentDifference * feedDifference) + shoulderFeedMap[i-1][1];
      }
    }

    for(int i = 1; i < elbowFeedMap.length; i++) {
      if(elbowFeedMap[i][0] > elbowPosition) {
        double percentDifference = (elbowPosition - elbowFeedMap[i-1][0]) / (elbowFeedMap[i][0] - elbowFeedMap[i-1][0]);
        double feedDifference = elbowFeedMap[i][1] - elbowFeedMap[i-1][1];
        elbowFeed = (percentDifference * feedDifference) + elbowFeedMap[i-1][1];
      }
    }

    setClimberFeedForward(shoulderFeed, elbowFeed);
  }

  public void setMotors(double shoulderOutput, double elbowOutput) {
    m_shoulder.set(shoulderOutput);
    m_elbow.set(elbowOutput);
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

    shoulderPosition *= ClimberConstants.SHOULDER_GB_RATIO;
    elbowPosition *= ClimberConstants.ELBOW_GB_RATIO;

    // shoulderPosition += m_shoulderOffset;
    // elbowPosition += m_elbowOffset;

    m_shoulder.set(TalonFXControlMode.Position, shoulderPosition);
    m_elbow.set(TalonFXControlMode.Position, elbowPosition);
  }

  public void controlWithInput(double xInput, double yInput) {
    tPoint.x += xInput * ClimberConstants.MOVE_SPEED;
    tPoint.y += yInput * ClimberConstants.MOVE_SPEED;
  }

  @Override
  public void periodic() {
    double[] jointAngles = getJointAngles(tPoint, 0.d);
    setJointAngles(jointAngles);
  }

  /**
   * Gets joint angles for climber arm
   * {@code targetPoint.x} and {@code targetPoint.y} are set in the xy plane of the climber, accounting for the
   * rotation of the robot. Both parameters should be in cm.
   * Does not set the motors automatically
   * <p><p>
   * IK resource: https://devforum.roblox.com/t/2-joint-2-limb-inverse-kinematics/252399
   * 
   * @param targetPoint Target xy point for the climber arm to go to
   * @param tiltAngle The tilt of the robot
   * @returns [shoulderAngle, elbowAngle] in radians */
   public static double[] getJointAngles(Point targetPoint, double tiltAngle) {
    double [] angles = new double[2];
  
    double mag = Math.hypot(targetPoint.x, targetPoint.y);
    if(mag >= ClimberConstants.MAX_ARM_LENGTH) {
      targetPoint.x = (targetPoint.x / mag) * ClimberConstants.MAX_ARM_LENGTH;
      targetPoint.y = (targetPoint.y / mag) * ClimberConstants.MAX_ARM_LENGTH;
      mag = ClimberConstants.MAX_ARM_LENGTH;
    } else if(mag < ClimberConstants.MIN_ARM_LENGTH && mag != 0) {
      targetPoint.x = (targetPoint.x / mag) * ClimberConstants.MIN_ARM_LENGTH;
      targetPoint.y = (targetPoint.y / mag) * ClimberConstants.MIN_ARM_LENGTH;
      mag = ClimberConstants.MIN_ARM_LENGTH;
    } else if(mag < ClimberConstants.MIN_ARM_LENGTH) {
      targetPoint.x = ClimberConstants.MIN_ARM_LENGTH;
      targetPoint.y = 0.d;
      mag = ClimberConstants.MIN_ARM_LENGTH;
    }
  
    // The angle to the target point
    double theta;
    if(targetPoint.x == 0.d) {
      theta = Math.PI/2.d; // TODO rename variable
    } else {
      theta = Math.atan(targetPoint.y / targetPoint.x);
    }
    theta += tiltAngle;
    
    if(targetPoint.x < 0.d)
        theta += Math.PI;
    
    
    // Correct target position for tilt
    targetPoint.x = Math.cos(theta) * mag;
    targetPoint.y = Math.sin(theta) * mag;
    
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
      double xDiff = targetPoint.x - ClimberConstants.LOWER_ARM_LENGTH;
      // System.out.println("xDiff: " + xDiff);
  
      elbowAngle = Math.atan(-targetPoint.y / xDiff);
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
}
