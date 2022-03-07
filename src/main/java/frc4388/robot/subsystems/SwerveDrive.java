// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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

  Translation2d m_frontLeftLocation = new Translation2d(Units.inchesToMeters(halfHeight),
      Units.inchesToMeters(halfWidth));
  Translation2d m_frontRightLocation = new Translation2d(Units.inchesToMeters(halfHeight),
      Units.inchesToMeters(-halfWidth));
  Translation2d m_backLeftLocation = new Translation2d(Units.inchesToMeters(-halfHeight),
      Units.inchesToMeters(halfWidth));
  Translation2d m_backRightLocation = new Translation2d(Units.inchesToMeters(-halfHeight),
      Units.inchesToMeters(-halfWidth));

  public SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
      m_backLeftLocation, m_backRightLocation);

  public SwerveModule[] modules;
  public WPI_PigeonIMU m_gyro;
  protected FusionStatus fstatus = new FusionStatus();

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  public SwerveDrivePoseEstimator m_poseEstimator;
  public SwerveDriveOdometry m_odometry;

  public double speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  public boolean ignoreAngles;
  public Rotation2d rotTarget = new Rotation2d();
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  private final Field2d m_field = new Field2d();

  public SwerveDrive(SwerveModule leftFront, SwerveModule leftBack, SwerveModule rightFront, SwerveModule rightBack,
      WPI_PigeonIMU gyro) {

    m_leftFront = leftFront;
    m_leftBack = leftBack;
    m_rightFront = rightFront;
    m_rightBack = rightBack;
    m_gyro = gyro;

    modules = new SwerveModule[] { m_leftFront, m_rightFront, m_leftBack, m_rightBack };

    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_gyro.getRotation2d(),
        new Pose2d(),
        m_kinematics,
        VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(1)),
        VecBuilder.fill(Units.degreesToRadians(1)),
        VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(1)));

    m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);
  }

  // https://github.com/ZachOrr/MK3-Swerve-Example
  /**
   * Method to drive the robot using joystick info.
   *
   * @param speeds[0]     Speed of the robot in the x direction (forward).
   * @param speeds[1]     Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveWithInput(double speedX, double speedY, double rot, boolean fieldRelative) {
    if (speedX == 0 && speedY == 0 && rot == 0)
      ignoreAngles = true;
    else
      ignoreAngles = false;
    Translation2d speed = new Translation2d(-speedX, speedY);
    double mag = speed.getNorm();
    speed = speed.times(mag * speedAdjust);

    double xSpeedMetersPerSecond = -speed.getX();
    double ySpeedMetersPerSecond = speed.getY();
    chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond,
            rot * SwerveDriveConstants.ROTATION_SPEED, m_gyro.getRotation2d())
        : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond,
            rot * SwerveDriveConstants.ROTATION_SPEED);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  public void driveWithInput(double leftX, double leftY, double rightX, double rightY, boolean fieldRelative) {
    ignoreAngles = leftX == 0 && leftY == 0 && rightX == 0 && rightY == 0;
    Translation2d speed = new Translation2d(-leftX, leftY);
    speed = speed.times(speed.getNorm() * speedAdjust);
    if (Math.abs(rightX) > OIConstants.RIGHT_AXIS_DEADBAND || Math.abs(rightY) > OIConstants.RIGHT_AXIS_DEADBAND)
      rotTarget = new Rotation2d(rightX, -rightY).minus(new Rotation2d(0, 1));
    double rot = rotTarget.minus(m_gyro.getRotation2d()).getRadians();
    double xSpeedMetersPerSecond = -speed.getX();
    double ySpeedMetersPerSecond = speed.getY();
    chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond,
            rot * SwerveDriveConstants.ROTATION_SPEED, m_gyro.getRotation2d())
        : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rightX * SwerveDriveConstants.ROTATION_SPEED);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(
        chassisSpeeds);
    setModuleStates(states);
  }

  /**
   * Set each module of the swerve drive to the corresponding desired state.
   * 
   * @param desiredStates Array of module states to set.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SEC));
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = desiredStates[i];
      module.setDesiredState(state, ignoreAngles);
    }
    // modules[0].setDesiredState(desiredStates[0], false);
  }

  @Override
  public void periodic() {

    updateOdometry();
    updateSmartDash();

    SmartDashboard.putNumber("Pigeon Yaw", m_gyro.getYaw());

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    super.periodic();
  }

  private void updateSmartDash() {
    // odometry
    SmartDashboard.putNumber("Odometry: X", getOdometry().getX());
    SmartDashboard.putNumber("Odometry: Y", getOdometry().getY());
    SmartDashboard.putNumber("Odometry: θ", getOdometry().getRotation().getDegrees());

    // chassis speeds
    // TODO: find the actual max velocity in m/s of the robot in fast mode to have accurate chassis speeds
    SmartDashboard.putNumber("Chassis Vel: X", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis Vel: Y", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis Vel: ω", chassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Gets the current chassis speeds in m/s and rad/s.
   * @return Current chassis speeds (vx, vy, ω)
   */
  public double[] getChassisSpeeds() {
    return new double[] {chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond};
  }

  /**
   * Gets the current pose of the robot.
   * 
   * @return Robot's current pose.
   */
  public Pose2d getOdometry() {
    // return m_odometry.getPoseMeters();
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the current gyro using regression formula.
   * 
   * @return Rotation2d object holding current gyro in radians
   */
  public Rotation2d getRegGyro() {
    double regCur = 0.6552670369 + m_gyro.getRotation2d().getDegrees() * 0.9926871527;
    return new Rotation2d(regCur * Math.PI / 180);
  }

  /**
   * Resets the odometry of the robot to the given pose.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_poseEstimator.update( getRegGyro(),
                            modules[0].getState(), 
                            modules[1].getState(), 
                            modules[2].getState(), 
                            modules[3].getState());
  }
  
  
  /**
   * Resets pigeon.
   */
  public void resetGyro() {
    m_gyro.reset();
    rotTarget = new Rotation2d(0);
  }

  /**
   * Stop all four swerve modules.
   */
  public void stopModules() {
    modules[0].stop();
    modules[1].stop();
    modules[2].stop();
    modules[3].stop();
  }

  /**
   * Switches speed modes.
   * 
   * @param shift True if fast mode, false if slow mode.
   */
  public void highSpeed(boolean shift) {
    if (shift) {
      speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_FAST;
    } else {
      speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
    }
  }
}