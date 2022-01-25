// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.RobotMap;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;

public class SwerveDrive extends SubsystemBase {
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
  

  // setSwerveGains();
      
  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public SwerveModule[] modules;
  public WPI_PigeonIMU m_gyro;

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private SwerveDrivePoseEstimator m_poseEstimator;

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
        new SwerveModule(m_leftFrontWheelMotor, m_leftFrontSteerMotor, m_leftFrontEncoder, SwerveDriveConstants.LEFT_FRONT_ENCODER_OFFSET), // Front Left
        new SwerveModule(m_rightFrontWheelMotor, m_rightFrontSteerMotor, m_rightFrontEncoder, SwerveDriveConstants.RIGHT_FRONT_ENCODER_OFFSET), // Front Right
        new SwerveModule(m_leftBackWheelMotor, m_leftBackSteerMotor, m_leftBackEncoder, SwerveDriveConstants.LEFT_BACK_ENCODER_OFFSET), // Back Left
        new SwerveModule(m_rightBackWheelMotor, m_rightBackSteerMotor, m_rightBackEncoder, SwerveDriveConstants.RIGHT_BACK_ENCODER_OFFSET)  // Back Right
      };
    m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_gyro.getRotation2d(),
          new Pose2d(),
          m_kinematics,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(Units.degreesToRadians(0.01)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
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
          m_kinematics.toSwerveModuleStates(
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
    //System.err.println(m_gyro.getFusedHeading() +"    aaa");
    updateOdometry();
    // m_gyro.setFusedHeadingToCompass();
    // m_gyro.setYawToCompass();
    super.periodic();
  }

  /**
   * Gets the distance between two given poses.
   * @param p1 The first pose.
   * @param p2 The second pose.
   * @return Absolute distance between p1 and p2.
   */
  public double distBtwPoses(Pose2d p1, Pose2d p2) {
    return Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
  }

  /**
   * Returns a scalar from your distance to the hub to your target distance.
   * 
   * @param target_dist The target distance.
   * @return A scalar that multiplies your distance from the hub to get your target distance.
   */
  public Pose2d poseGivenDist(double target_dist) {
    Pose2d p1 = m_poseEstimator.getEstimatedPosition();
    Pose2d p2 = SwerveDriveConstants.HUB_POSE;

    double scalar = target_dist/distBtwPoses(p1, p2);
    Pose2d new_pose = new Pose2d(p1.getX() * scalar, p1.getY() * scalar, p1.getRotation());

    return new_pose;
  }

  /**
   * Gets the current pose of the robot.
   * @return Robot's current pose.
   */
  public Pose2d getOdometry() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry of the robot to (x=0, y=0, theta=0).
   */
  public void resetOdometry() {
    m_poseEstimator.resetPosition(new Pose2d(0, 0, new Rotation2d(0)), m_gyro.getRotation2d());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update( m_gyro.getRotation2d(), 
                            modules[0].getState(), 
                            modules[1].getState(), 
                            modules[2].getState(), 
                            modules[3].getState());
  
      // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
      // a real robot, this must be calculated based either on latency or timestamps.
      // m_poseEstimator.addVisionMeasurement(
      //         m_poseEstimator.getEstimatedPosition(),
      //     Timer.getFPGATimestamp() - 0.1);
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