// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.nio.ByteBuffer;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;
import frc4388.utility.controller.XboxController;

public class SwerveDrive extends SubsystemBase {
  SwerveDriveKinematics m_kinematics;
  private WPI_TalonFX m_leftFrontSteerMotor;
  private WPI_TalonFX m_leftFrontWheelMotor;
  private WPI_TalonFX m_rightFrontSteerMotor;
  private WPI_TalonFX m_rightFrontWheelMotor;
  private WPI_TalonFX m_leftBackSteerMotor;
  private WPI_TalonFX m_leftBackWheelMotor;
  private WPI_TalonFX m_rightBackSteerMotor;
  private WPI_TalonFX m_rightBackWheelMotor;
  private CANCoder m_leftFrontEncoder; 
  private CANCoder m_rightFrontEncoder;
  private CANCoder m_leftBackEncoder;
  private CANCoder m_rightBackEncoder;    
  double halfWidth = SwerveDriveConstants.WIDTH / 2.d;
  double halfHeight = SwerveDriveConstants.HEIGHT / 2.d;
  public static Gains m_swerveGains = SwerveDriveConstants.SWERVE_GAINS;


  Translation2d m_frontLeftLocation = new Translation2d(Units.inchesToMeters(halfHeight), Units.inchesToMeters(halfWidth));
  Translation2d m_frontRightLocation = new Translation2d(Units.inchesToMeters(halfHeight), Units.inchesToMeters(-halfWidth));
  Translation2d m_backLeftLocation = new Translation2d(Units.inchesToMeters(-halfHeight), Units.inchesToMeters(halfWidth));
  Translation2d m_backRightLocation = new Translation2d(Units.inchesToMeters(-halfHeight), Units.inchesToMeters(-halfWidth));
  
  SwerveModule m_frontLeft = new SwerveModule(m_leftFrontWheelMotor, m_leftFrontSteerMotor, m_leftFrontEncoder, SwerveDriveConstants.LEFT_FRONT_ENCODER_OFFSET);
  SwerveModule m_frontRight = new SwerveModule(m_rightFrontWheelMotor, m_rightFrontSteerMotor, m_rightFrontEncoder, SwerveDriveConstants.RIGHT_FRONT_ENCODER_OFFSET);
  SwerveModule m_backLeft = new SwerveModule(m_leftBackWheelMotor, m_leftBackSteerMotor, m_leftBackEncoder, SwerveDriveConstants.LEFT_BACK_ENCODER_OFFSET);
  SwerveModule m_backRight = new SwerveModule(m_rightBackWheelMotor, m_rightBackSteerMotor, m_rightBackEncoder, SwerveDriveConstants.RIGHT_BACK_ENCODER_OFFSET);

  // setSwerveGains();
      
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public SwerveModule[] modules;
  public WPI_PigeonIMU m_gyro;

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_gyro.getRotation2d(),
          new Pose2d(),
          m_kinematics,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(Units.degreesToRadians(0.01)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public double speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  public boolean ignoreAngles;

  public SwerveDrive(WPI_TalonFX leftFrontSteerMotor,WPI_TalonFX leftFrontWheelMotor,WPI_TalonFX rightFrontSteerMotor,WPI_TalonFX rightFrontWheelMotor,
  WPI_TalonFX leftBackSteerMotor,WPI_TalonFX leftBackWheelMotor,WPI_TalonFX rightBackSteerMotor,WPI_TalonFX rightBackWheelMotor, CANCoder leftFrontEncoder,
  CANCoder rightFrontEncoder,
  CANCoder leftBackEncoder,
  CANCoder rightBackEncoder, WPI_PigeonIMU gyro)
  {
      m_leftFrontSteerMotor = leftFrontSteerMotor;
      m_leftFrontWheelMotor = leftFrontWheelMotor;
      m_rightFrontSteerMotor = rightFrontSteerMotor;
      m_rightFrontWheelMotor = rightFrontWheelMotor;
      m_leftBackSteerMotor = leftBackSteerMotor;
      m_leftBackWheelMotor = leftBackWheelMotor;
      m_rightBackSteerMotor = rightBackSteerMotor;
      m_rightBackWheelMotor = rightBackWheelMotor;
      m_leftFrontEncoder = leftFrontEncoder; 
      m_rightFrontEncoder = rightFrontEncoder;
      m_leftBackEncoder = leftBackEncoder;
      m_rightBackEncoder = rightBackEncoder; 
      m_gyro = gyro;

      modules = new SwerveModule[] {
          m_frontLeft, // Front Left
          m_frontRight, // Front Right
          m_backLeft, // Back Left
          m_backRight  // Back Right
      };
      m_gyro.reset(); 
  }
//https://github.com/ZachOrr/MK3-Swerve-Example
 /**
 * Method to drive the robot using joystick info.
 *
 * @param speeds[0] Speed of the robot in the x direction (forward).
 * @param speeds[1] Speed of the robot in the y direction (sideways).
 * @param rot Angular rate of the robot.
 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
 */
  public void driveWithInput(double[] speeds, double rot, boolean fieldRelative)
  {
      if (speeds[0] == 0 && speeds[1] == 0 && rot == 0) ignoreAngles = true;
      else ignoreAngles = false;
      speeds[0] *= speeds[0] * speeds[0];
      speeds[1] *= speeds[1] * speeds[1];

      double xSpeedMetersPerSecond = -speeds[0] * speedAdjust;
      double ySpeedMetersPerSecond = speeds[1] * speedAdjust;
      SwerveModuleState[] states =
          kinematics.toSwerveModuleStates(
              fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot * SwerveDriveConstants.ROTATION_SPEED, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot * SwerveDriveConstants.ROTATION_SPEED));
       SwerveDriveKinematics.desaturateWheelSpeeds(states, Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SEC));
       for (int i = 0; i < states.length; i++) {
          SwerveModule module = modules[i];
          SwerveModuleState state = states[i];
          module.setDesiredState(state, ignoreAngles);
    }
  }
  
  @Override
  public void periodic() {
    System.err.println(m_gyro.getFusedHeading() +"    aaa");
    // m_gyro.setFusedHeadingToCompass();
    // m_gyro.setYawToCompass();
    // m_gyro.getRotation2d();
    super.periodic();
  }

  /**
   * Get a "noisy" fake global pose reading.
   *
   * @param estimatedRobotPose The robot pose.
   */
  // TEST FAKE CODE FROM EXAMPLE
  public static Pose2d getEstimatedGlobalPose(Pose2d estimatedRobotPose) {
    var rand =
        StateSpaceUtil.makeWhiteNoiseVector(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    return new Pose2d(
        estimatedRobotPose.getX() + rand.get(0, 0),
        estimatedRobotPose.getY() + rand.get(1, 0),
        estimatedRobotPose.getRotation().plus(new Rotation2d(rand.get(2, 0))));
  }

  /**
   * Returns a scalar from your distance to the hub to your target distance.
   * 
   * @param target_dist The target distance.
   * @return A scalar that multiplies your distance from the hub to get your target distance.
   */
  public double getScalarForLine(double target_dist) {
    Pose2d p1 = m_poseEstimator.getEstimatedPosition();
    Pose2d p2 = SwerveDriveConstants.HUB_POSE;

    double dist = Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));

    return target_dist/dist;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update( m_gyro.getRotation2d(), 
                            m_frontLeft.getState(), 
                            m_frontRight.getState(), 
                            m_backLeft.getState(), 
                            m_backRight.getState());
  
      // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
      // a real robot, this must be calculated based either on latency or timestamps.
      m_poseEstimator.addVisionMeasurement(
          SwerveDrive.getEstimatedGlobalPose(
              m_poseEstimator.getEstimatedPosition()),
          Timer.getFPGATimestamp() - 0.1);
    }

  public void highSpeed(boolean shift){
    if (shift){
      speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_FAST;
    }
    else{
      speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
    }
  }
}