// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule m_leftFront;
  private SwerveModule m_leftBack;
  private SwerveModule m_rightFront;
  private SwerveModule m_rightBack;

  double halfWidth = SwerveDriveConstants.WIDTH / 2.d;
  double halfHeight = SwerveDriveConstants.HEIGHT / 2.d;

  public static Gains m_swerveGains = SwerveDriveConstants.SWERVE_GAINS;

  Translation2d m_frontLeftLocation = new Translation2d(Units.inchesToMeters(halfHeight), Units.inchesToMeters(halfWidth));
  Translation2d m_frontRightLocation = new Translation2d(Units.inchesToMeters(halfHeight), Units.inchesToMeters(-halfWidth));
  Translation2d m_backLeftLocation = new Translation2d(Units.inchesToMeters(-halfHeight), Units.inchesToMeters(halfWidth));
  Translation2d m_backRightLocation = new Translation2d(Units.inchesToMeters(-halfHeight), Units.inchesToMeters(-halfWidth));
      
  public SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public SwerveModule[] modules;
  public WPI_PigeonIMU m_gyro;
  protected FusionStatus fstatus = new FusionStatus();

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  public SwerveDrivePoseEstimator m_poseEstimator;

  public double speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  public boolean ignoreAngles;

  private final Field2d m_field = new Field2d();

  public SwerveDrive(SwerveModule leftFront, SwerveModule leftBack, SwerveModule rightFront, SwerveModule rightBack, WPI_PigeonIMU gyro) {
      // m_leftFrontSteerMotor = leftFrontSteerMotor;
      // m_leftFrontWheelMotor = leftFrontWheelMotor;
      // m_rightFrontSteerMotor = rightFrontSteerMotor;
      // m_rightFrontWheelMotor = rightFrontWheelMotor;
      // m_leftBackSteerMotor = leftBackSteerMotor;
      // m_leftBackWheelMotor = leftBackWheelMotor;
      // m_rightBackSteerMotor = rightBackSteerMotor;
      // m_rightBackWheelMotor = rightBackWheelMotor;
      // m_leftFrontEncoder = leftFrontEncoder; 
      // m_rightFrontEncoder = rightFrontEncoder;
      // m_leftBackEncoder = leftBackEncoder;
      // m_rightBackEncoder = rightBackEncoder; 
    m_leftFront = leftFront;
    m_leftBack = leftBack;
    m_rightFront = rightFront;
    m_rightBack = rightBack;
    m_gyro = gyro;

      // modules = new SwerveModule[] {
      //   new SwerveModule(m_leftFrontWheelMotor, m_leftFrontSteerMotor, m_leftFrontEncoder, SwerveDriveConstants.LEFT_FRONT_ENCODER_OFFSET), // Front Left
      //   new SwerveModule(m_rightFrontWheelMotor, m_rightFrontSteerMotor, m_rightFrontEncoder, SwerveDriveConstants.RIGHT_FRONT_ENCODER_OFFSET), // Front Right
      //   new SwerveModule(m_leftBackWheelMotor, m_leftBackSteerMotor, m_leftBackEncoder, SwerveDriveConstants.LEFT_BACK_ENCODER_OFFSET), // Back Left
      //   new SwerveModule(m_rightBackWheelMotor, m_rightBackSteerMotor, m_rightBackEncoder, SwerveDriveConstants.RIGHT_BACK_ENCODER_OFFSET)  // Back Right
      // };

    modules = new SwerveModule[] {m_leftFront, m_rightFront, m_leftBack, m_rightBack};

    m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_gyro.getRotation2d(),
          new Pose2d(),
          m_kinematics,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(Units.degreesToRadians(0.01)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    m_gyro.reset(); 
    SmartDashboard.putData("Field", m_field);
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
  public void driveWithInput(double speedX, double speedY, double rot, boolean fieldRelative)
  {
    if (speedX == 0 && speedY == 0 && rot == 0) ignoreAngles = true;
    else ignoreAngles = false;
    Translation2d speed = new Translation2d(speedX, speedY);
    double mag = speed.getNorm();
    speed = speed.times(mag * speedAdjust);

    double xSpeedMetersPerSecond = -speed.getX();
    double ySpeedMetersPerSecond = speed.getY();
    SwerveModuleState[] states =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot * SwerveDriveConstants.ROTATION_SPEED, m_gyro.getRotation2d())
              : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot * SwerveDriveConstants.ROTATION_SPEED));
    setModuleStates(states);
  }
  private Rotation2d rotTarget = new Rotation2d();
  public void driveWithInput(double leftX, double leftY, double rightX, double rightY, boolean fieldRelative)
  {
    ignoreAngles = leftX == 0 && leftY == 0 && rightX == 0 && rightY == 0;
    Translation2d speed = new Translation2d(leftX, leftY);
    speed = speed.times(speed.getNorm() * speedAdjust);
    if (Math.abs(rightX) > OIConstants.RIGHT_AXIS_DEADBAND || Math.abs(rightY) > OIConstants.RIGHT_AXIS_DEADBAND)
      rotTarget = new Rotation2d(rightX, -rightY).minus(new Rotation2d(0, 1));
    double rot = rotTarget.minus(m_gyro.getRotation2d()).getRadians();
    double xSpeedMetersPerSecond = -speed.getX();
    double ySpeedMetersPerSecond = speed.getY();
    SwerveModuleState[] states =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot * SwerveDriveConstants.ROTATION_SPEED, m_gyro.getRotation2d())
              : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rightX * SwerveDriveConstants.ROTATION_SPEED));
    setModuleStates(states);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SEC));
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = desiredStates[i];
      module.setDesiredState(state, false);
    }
  }
  
  @Override
  public void periodic() {
    //System.err.println(m_gyro.getFusedHeading() +"    aaa");
    updateOdometry();
    SmartDashboard.putNumber("Pigeon Fused Heading", m_gyro.getFusedHeading(fstatus));
    SmartDashboard.putNumber("Pigeon Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("Pigeon Get Angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Pigeon Rotation 2D", m_gyro.getRotation2d().getDegrees());
    SmartDashboard.putStringArray("Fusion Status", new String[] {"Is Fusing: "+fstatus.bIsFusing, "Is Valid: "+fstatus.bIsValid, "Heading: "+fstatus.heading});

    // m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 1, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    // m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 1, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
    // m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 1, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
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
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(pose, m_gyro.getRotation2d());
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

  public void stopModules() {
    modules[0].stop();
    modules[1].stop();
    modules[2].stop();
    modules[3].stop();
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