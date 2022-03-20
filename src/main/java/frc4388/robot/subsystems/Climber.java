// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.Arrays;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;
import frc4388.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private static double[][] shoulderFeedMap;
  private static double[][] elbowFeedMap;

  public static void configureFeedMaps() {
    
  }

  private WPI_TalonFX m_shoulder;
  private WPI_TalonFX m_elbow;
  private WPI_Pigeon2 m_gyro;

  private Point tPoint;

  private double[] tJointAngles;
  private double[] tJointSpeeds;

  private boolean groundRelative;
  private double shoulderSpeedLimiter;
  private double elbowSpeedLimiter;

  /** Creates a new ClimberRewrite. */
  public Climber(WPI_TalonFX shoulder, WPI_TalonFX elbow, WPI_Pigeon2 gyro, boolean _groundRelative) {
    m_shoulder = shoulder;
    m_shoulder.configFactoryDefault();
    m_shoulder.setNeutralMode(NeutralMode.Brake);

    m_elbow = elbow;
    m_elbow.configFactoryDefault();
    m_elbow.setNeutralMode(NeutralMode.Brake);

    // setClimberPositionGains();
    setClimberVelocityGains();
    useVelocityGains();

    tJointAngles = new double[] {m_shoulder.getSelectedSensorPosition(), m_elbow.getSelectedSensorPosition()};

    // shoulderStartPosition = m_shoulder.getSelectedSensorPosition();
    // elbowStartPosition = m_elbow.getSelectedSensorPosition();
    // m_shoulder.setSelectedSensorPosition(((ClimberConstants.SHOULDER_RESTING_ANGLE * (Constants.TICKS_PER_ROTATION_FX/2.d)) / Math.PI) * ClimberConstants.SHOULDER_GB_RATIO);
    // m_elbow.setSelectedSensorPosition(((ClimberConstants.ELBOW_RESTING_ANGLE * (Constants.TICKS_PER_ROTATION_FX/2.d)) / Math.PI) * ClimberConstants.SHOULDER_GB_RATIO);

    m_elbow.configForwardSoftLimitThreshold(ClimberConstants.ELBOW_FORWARD_SOFT_LIMIT);
    m_elbow.configForwardSoftLimitEnable(true);
    // m_elbow.configReverseSoftLimitThreshold(ClimberConstants.ELBOW_SOFT_LIMIT_REVERSE);
    // m_elbow.configReverseSoftLimitEnable(true);

    m_shoulder.configForwardSoftLimitThreshold(ClimberConstants.SHOULDER_FORWARD_SOFT_LIMIT);
    m_shoulder.configForwardSoftLimitEnable(true);
    m_shoulder.configReverseSoftLimitThreshold(ClimberConstants.SHOULDER_REVERSE_SOFT_LIMIT);
    m_shoulder.configReverseSoftLimitEnable(false);

    m_shoulder.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_elbow.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_shoulder.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    
    m_shoulder.overrideLimitSwitchesEnable(true);
    m_elbow.overrideLimitSwitchesEnable(true);

    tPoint = new Point();

    if(groundRelative)
      m_gyro = gyro;
    
    groundRelative = _groundRelative;
    this.elbowSpeedLimiter = 1.0;
  }

  // public void setClimberPositionGains() {
  //   m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_POSITION_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
  //   m_shoulder.config_kF(ClimberConstants.SHOULDER_POSITION_SLOT_IDX, ClimberConstants.SHOULDER_POSITION_GAINS.kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
  //   m_shoulder.config_kP(ClimberConstants.SHOULDER_POSITION_SLOT_IDX, ClimberConstants.SHOULDER_POSITION_GAINS.kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
  //   m_shoulder.config_kI(ClimberConstants.SHOULDER_POSITION_SLOT_IDX, ClimberConstants.SHOULDER_POSITION_GAINS.kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
  //   m_shoulder.config_kD(ClimberConstants.SHOULDER_POSITION_SLOT_IDX, ClimberConstants.SHOULDER_POSITION_GAINS.kD, ClimberConstants.CLIMBER_TIMEOUT_MS);

  //   m_elbow.selectProfileSlot(ClimberConstants.ELBOW_POSITION_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
  //   m_elbow.config_kF(ClimberConstants.ELBOW_POSITION_SLOT_IDX, ClimberConstants.ELBOW_POSITION_GAINS.kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
  //   m_elbow.config_kP(ClimberConstants.ELBOW_POSITION_SLOT_IDX, ClimberConstants.ELBOW_POSITION_GAINS.kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
  //   m_elbow.config_kI(ClimberConstants.ELBOW_POSITION_SLOT_IDX, ClimberConstants.ELBOW_POSITION_GAINS.kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
  //   m_elbow.config_kD(ClimberConstants.ELBOW_POSITION_SLOT_IDX, ClimberConstants.ELBOW_POSITION_GAINS.kD, ClimberConstants.CLIMBER_TIMEOUT_MS);
  // }

  // public void usePositionGains() {
  //   m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_POSITION_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
  //   m_elbow.selectProfileSlot(ClimberConstants.ELBOW_POSITION_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
  // }

  public void setClimberVelocityGains() {
    m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_VELOCITY_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
    m_shoulder.config_kF(ClimberConstants.SHOULDER_VELOCITY_SLOT_IDX, ClimberConstants.SHOULDER_VELOCITY_GAINS.kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kP(ClimberConstants.SHOULDER_VELOCITY_SLOT_IDX, ClimberConstants.SHOULDER_VELOCITY_GAINS.kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kI(ClimberConstants.SHOULDER_VELOCITY_SLOT_IDX, ClimberConstants.SHOULDER_VELOCITY_GAINS.kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_shoulder.config_kD(ClimberConstants.SHOULDER_VELOCITY_SLOT_IDX, ClimberConstants.SHOULDER_VELOCITY_GAINS.kD, ClimberConstants.CLIMBER_TIMEOUT_MS);

    m_elbow.selectProfileSlot(ClimberConstants.ELBOW_VELOCITY_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
    m_elbow.config_kF(ClimberConstants.ELBOW_VELOCITY_SLOT_IDX, ClimberConstants.ELBOW_VELOCITY_GAINS.kF, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kP(ClimberConstants.ELBOW_VELOCITY_SLOT_IDX, ClimberConstants.ELBOW_VELOCITY_GAINS.kP, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kI(ClimberConstants.ELBOW_VELOCITY_SLOT_IDX, ClimberConstants.ELBOW_VELOCITY_GAINS.kI, ClimberConstants.CLIMBER_TIMEOUT_MS);
    m_elbow.config_kD(ClimberConstants.ELBOW_VELOCITY_SLOT_IDX, ClimberConstants.ELBOW_VELOCITY_GAINS.kD, ClimberConstants.CLIMBER_TIMEOUT_MS);
  }

  public void useVelocityGains() {
    m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_VELOCITY_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
    m_elbow.selectProfileSlot(ClimberConstants.ELBOW_VELOCITY_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
  }

  // public void setClimberFeedForward(double shoulderF, double elbowF) {
  //   m_shoulder.selectProfileSlot(ClimberConstants.SHOULDER_SLOT_IDX, ClimberConstants.SHOULDER_PID_LOOP_IDX);
  //   m_shoulder.config_kF(ClimberConstants.SHOULDER_SLOT_IDX, shoulderF, ClimberConstants.CLIMBER_TIMEOUT_MS);

  //   m_elbow.selectProfileSlot(ClimberConstants.ELBOW_SLOT_IDX, ClimberConstants.ELBOW_PID_LOOP_IDX);
  //   m_elbow.config_kF(ClimberConstants.ELBOW_SLOT_IDX, elbowF, ClimberConstants.CLIMBER_TIMEOUT_MS);
  // }

  // public void compensateFeedForArmAngles() {
  //   double shoulderPosition = m_shoulder.getSelectedSensorPosition();
  //   double elbowPosition = m_elbow.getSelectedSensorPosition();

  //   double shoulderFeed = 0;
  //   double elbowFeed = 0;

  //   for(int i = 1; i < shoulderFeedMap.length; i++) {
  //     if(shoulderFeedMap[i][0] > shoulderPosition) {
  //       double percentDifference = (shoulderPosition - shoulderFeedMap[i-1][0]) / (shoulderFeedMap[i][0] - shoulderFeedMap[i-1][0]);
  //       double feedDifference = shoulderFeedMap[i][1] - shoulderFeedMap[i-1][1];
  //       shoulderFeed = (percentDifference * feedDifference) + shoulderFeedMap[i-1][1];
  //     }
  //   }

  //   for(int i = 1; i < elbowFeedMap.length; i++) {
  //     if(elbowFeedMap[i][0] > elbowPosition) {
  //       double percentDifference = (elbowPosition - elbowFeedMap[i-1][0]) / (elbowFeedMap[i][0] - elbowFeedMap[i-1][0]);
  //       double feedDifference = elbowFeedMap[i][1] - elbowFeedMap[i-1][1];
  //       elbowFeed = (percentDifference * feedDifference) + elbowFeedMap[i-1][1];
  //     }
  //   }

  //   setClimberFeedForward(shoulderFeed, elbowFeed);
  // }

  public void setMotors(double shoulderOutput, double elbowOutput) {
    m_shoulder.set(shoulderOutput * ClimberConstants.INPUT_MULTIPLIER );//* this.shoulderSpeedLimiter);
    m_elbow.set(elbowOutput * ClimberConstants.INPUT_MULTIPLIER);// * this.elbowSpeedLimiter);
  }

  public double[] getJointAngles() {
    return new double[] {
      (m_shoulder.getSelectedSensorPosition() * Math.PI) / (Constants.TICKS_PER_ROTATION_FX/2.d) / ClimberConstants.SHOULDER_GB_RATIO,
      (m_elbow.getSelectedSensorPosition() * Math.PI) / (Constants.TICKS_PER_ROTATION_FX/2.d) / ClimberConstants.ELBOW_GB_RATIO
    };
  }

  public void setJointAngles(double[] angles) {
    // System.out.println(Arrays.toString(angles));
    setJointAngles(angles[0], angles[1]);
  }

  public void setJointAngles(double shoulderAngle, double elbowAngle) {
    // shoulderAngle = shoulderAngle < ClimberConstants.SHOULDER_RESTING_ANGLE ? ClimberConstants.SHOULDER_RESTING_ANGLE : shoulderAngle;
    // elbowAngle = elbowAngle < ClimberConstants.ELBOW_RESTING_ANGLE ? ClimberConstants.ELBOW_RESTING_ANGLE : elbowAngle;

    // shoulderAngle = shoulderAngle > ClimberConstants.SHOULDER_MAX_ANGLE ? ClimberConstants.SHOULDER_MAX_ANGLE : shoulderAngle;
    // elbowAngle = elbowAngle > ClimberConstants.ELBOW_MAX_ANGLE ? ClimberConstants.ELBOW_MAX_ANGLE : elbowAngle;

    // // Convert radians to ticks
    // System.out.println("angles: " + shoulderAngle + ", " + elbowAngle);

    // double shoulderPosition = (shoulderAngle * (Constants.TICKS_PER_ROTATION_FX/2.d)) / Math.PI;
    // double elbowPosition = (elbowAngle * (Constants.TICKS_PER_ROTATION_FX/2.d)) / Math.PI;

    // shoulderPosition *= ClimberConstants.SHOULDER_GB_RATIO;
    // elbowPosition *= ClimberConstants.ELBOW_GB_RATIO;

    // // shoulderPosition += m_shoulderOffset;
    // // elbowPosition += m_elbowOffset;

    // m_shoulder.set(TalonFXControlMode.Position, shoulderAngle);
    // m_elbow.set(TalonFXControlMode.Position, elbowAngle);
  }

  public void setJointSpeeds(double[] speeds) {
    setJointSpeeds(speeds[0], speeds[1]);
  }
  /**
   * Velocity PID set for joints
   * @param shoulderSpeed
   * @param elbowSpeed
   */
  public void setJointSpeeds(double shoulderSpeed, double elbowSpeed) {
    m_shoulder.set(TalonFXControlMode.Velocity, shoulderSpeed);
    m_elbow.set(TalonFXControlMode.Velocity, elbowSpeed);
  }

  boolean movingPrev = false;
  boolean moving;
  /**
   * 
   * @param xInput Rate of change of X position of target point
   * @param yInput Rate of change of Y position of target point
   * @deprecated use controlJointsWithInput() instead
   */
  @Deprecated
  public void controlWithInput(double xInput, double yInput) {
    moving = xInput != 0 || yInput != 0;

    if(movingPrev != moving) {
      if(moving) {
        useVelocityGains();
        SmartDashboard.putString("Climber Gains", "Velocity");
      } else {
        // usePositionGains();
        tJointAngles = new double[] {m_shoulder.getSelectedSensorPosition(), m_elbow.getSelectedSensorPosition()};
        SmartDashboard.putString("Climber Gains", "Position");
      }
    }

    if(moving) {
      double[] jointSpeeds = new double[] {xInput * ClimberConstants.MOVE_SPEED, yInput * ClimberConstants.MOVE_SPEED};
      setJointSpeeds(jointSpeeds);
    } else {
      setJointAngles(tJointAngles);
    }

    movingPrev = moving;
  }

  public void controlJointsWithInput(double shoulderInput, double elbowInput) {
    tJointAngles[0] += shoulderInput * ClimberConstants.MOVE_SPEED * .02;
    tJointAngles[1] += elbowInput * ClimberConstants.MOVE_SPEED * .02;
  }

  int pCount = 0;
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Moving", moving);
    SmartDashboard.putBoolean("MovingPrev", movingPrev);
    SmartDashboard.putNumber("Elbow", m_elbow.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shoulder", m_shoulder.getSelectedSensorPosition());
    // double[] jointAngles = getTargetJointAngles(tPoint, 0.d);
    // if(pCount % 1 == 0)
    //   setJointAngles(tJointAngles);

    // pCount ++;

    // * speed limiting near ELBOW soft limits. tolerance (distance when ramping starts) is 20000 rotations. speed at hard limits is 0.2 (percent output).
    double currentElbowPos = this.m_elbow.getSelectedSensorPosition();
    double forwardElbowDistance = Math.abs(currentElbowPos - ClimberConstants.ELBOW_FORWARD_SOFT_LIMIT);
    double reverseElbowDistance = Math.abs(currentElbowPos - ClimberConstants.ELBOW_REVERSE_SOFT_LIMIT);

    if (forwardElbowDistance < ClimberConstants.ELBOW_SOFT_LIMIT_TOLERANCE) {
      this.elbowSpeedLimiter = 0.15 + (forwardElbowDistance * (1 / ClimberConstants.ELBOW_SOFT_LIMIT_TOLERANCE));
    }

    if (reverseElbowDistance < ClimberConstants.ELBOW_SOFT_LIMIT_TOLERANCE) {
      this.elbowSpeedLimiter = 0.15 + (reverseElbowDistance * (1 / ClimberConstants.ELBOW_SOFT_LIMIT_TOLERANCE));
    }

    if ((forwardElbowDistance > ClimberConstants.ELBOW_SOFT_LIMIT_TOLERANCE) && (reverseElbowDistance > ClimberConstants.ELBOW_SOFT_LIMIT_TOLERANCE)) {
      this.elbowSpeedLimiter = 1.0;
    }

    // * speed limiting near SHOULDER soft limits. tolerance (distance when ramping starts) is 20000 rotations. speed at hard limits is 0.2 (percent output).
    double currentShoulderPos = this.m_shoulder.getSelectedSensorPosition();
    double forwardShoulderDistance = Math.abs(currentShoulderPos - ClimberConstants.SHOULDER_FORWARD_SOFT_LIMIT);
    double reverseShoulderDistance = Math.abs(currentShoulderPos - ClimberConstants.SHOULDER_REVERSE_SOFT_LIMIT);

    if (forwardShoulderDistance < ClimberConstants.SHOULDER_SOFT_LIMIT_TOLERANCE) {
      this.shoulderSpeedLimiter = 0.15 + (forwardShoulderDistance * (1 / ClimberConstants.SHOULDER_SOFT_LIMIT_TOLERANCE));
    }

    if (reverseShoulderDistance < ClimberConstants.SHOULDER_SOFT_LIMIT_TOLERANCE) {
      this.shoulderSpeedLimiter = 0.15 + (reverseShoulderDistance * (1 / ClimberConstants.SHOULDER_SOFT_LIMIT_TOLERANCE));
    }

    if ((forwardShoulderDistance > ClimberConstants.SHOULDER_SOFT_LIMIT_TOLERANCE) && (reverseShoulderDistance > ClimberConstants.SHOULDER_SOFT_LIMIT_TOLERANCE)) {
      this.shoulderSpeedLimiter = 1.0;
    }
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
   * @return [shoulderAngle, elbowAngle] in radians 
   * @deprecated
   * */
    @Deprecated
   public static double[] getTargetJointAngles(Point targetPoint, double tiltAngle) {
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
  /**
   * Forward kinematics for climber
   * @param shoulderAngle in radians
   * @param elbowAngle in radians
   * @return Point in 2d of end effector
   */
  public static Point getClimberPosition(double shoulderAngle, double elbowAngle) {
    Point climberPoint = new Point(0, 0);

    climberPoint.x += Math.sin(shoulderAngle);
    climberPoint.y += Math.cos(shoulderAngle);

    climberPoint.x -= Math.sin(elbowAngle - shoulderAngle);
    climberPoint.y += Math.cos(elbowAngle - shoulderAngle);

    return climberPoint;
  }

  public static Point getClimberPosition(double[] jointAngles) {
    return getClimberPosition(jointAngles[0], jointAngles[1]);
  }

  public void setClimberSoftLimits(boolean set){
    m_elbow.configForwardSoftLimitEnable(set);
    m_shoulder.configForwardSoftLimitEnable(set);
  }

  public void setEncoders(double value){
    m_elbow.setSelectedSensorPosition(value);
    m_shoulder.setSelectedSensorPosition(value);
  }

  public double getCurrent(){
    return (m_shoulder.getSupplyCurrent() + m_elbow.getSupplyCurrent());
  }

}
