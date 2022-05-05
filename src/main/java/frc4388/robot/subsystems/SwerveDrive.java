package frc4388.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

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

public class SwerveDrive extends SubsystemBase {

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;
  private WPI_Pigeon2 m_gyro;

  private SwerveModule[] m_modules;
  private SwerveDriveOdometry m_odometry;
  private ChassisSpeeds m_chassisSpeeds;
  private boolean m_speedMode;
  private boolean m_ignoreAngles;
  private double m_targetHeading;

  public final Field2d m_field = new Field2d();

  public SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, WPI_Pigeon2 gyro) {
    m_frontLeft = frontLeft;
    m_frontRight = frontRight;
    m_backLeft = backLeft;
    m_backRight = backRight;
    m_gyro = gyro;

    m_modules = new SwerveModule[] { m_frontLeft, m_frontRight, m_backLeft, m_backRight };
    m_odometry = new SwerveDriveOdometry(SwerveDriveConstants.DRIVE_KINEMATICS, m_gyro.getRotation2d());
    m_chassisSpeeds = new ChassisSpeeds();

    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    updateOdometry();
    updateSmartDash();

    m_field.setRobotPose(getPoseMeters());
  }

  /**
   * Method to drive the robot using joystick info.
   * @link https://github.com/ZachOrr/MK3-Swerve-Example
   * @param speeds[0] Speed of the robot in the x direction (forward).
   * @param speeds[1] Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void driveWithInput(Translation2d speed, double rot, boolean fieldRelative) {
    m_ignoreAngles = (speed.getX() == 0) && (speed.getY() == 0) && (rot == 0);

    double magnitude = speed.getNorm();
    speed = speed.times(magnitude * getMaxSpeed());

    double xSpeedMetersPerSecond = speed.getX();
    double ySpeedMetersPerSecond = speed.getY();
    m_chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, -rot * SwerveDriveConstants.ROTATION_SPEED, new Rotation2d(Math.toRadians(m_gyro.getAngle() + 90))) : new ChassisSpeeds(ySpeedMetersPerSecond, -xSpeedMetersPerSecond, -rot * SwerveDriveConstants.ROTATION_SPEED);
    setModuleStates(SwerveDriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(m_chassisSpeeds));
  }

  public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {
    if (fieldRelative) {
      // Use the right joystick to set heading.
      if (rightStick.getNorm() > OIConstants.RIGHT_AXIS_DEADBAND) {
        /*
         Map the right jostick coordinates to a ccw+ heading.
        
         Right Stick    Polar Radians   Omega Radians
          ┌──˗y───┐       ╭─˖π/2──╮       ╭──˖0───╮  
         ˗x      ˖x  ->  ̠˖π      ̠˖0  -> ˖π/2    ˗π/2 
          └──˖y───┘       ╰─˗π/2──╯       ╰──˖π───╯  
            (x,y)        tan⁻¹(y/x)     tan⁻¹(-y/-x) 
        */
        m_targetHeading = Math.atan2(-rightStick.getX(), -rightStick.getY());
      }
      // Calculate the error between the the target heading and the current heading.
      double headingError = m_targetHeading - Math.toRadians(m_gyro.getYaw());

      // Use the left joystick to set speed. Apply a quadratic curve and the set max speed.
      Translation2d speed = leftStick.times(leftStick.getNorm() * getMaxSpeed());

      // Convert field-relative speeds to robot-relative speeds.
      m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speed.getX(), speed.getY(), headingError, new Rotation2d(Math.toRadians(m_gyro.getAngle() + 90)));
    } else {
      // Create robot-relative speeds.
      m_chassisSpeeds = new ChassisSpeeds(leftStick.getX(), leftStick.getY(), rightStick.getX() * SwerveDriveConstants.ROTATION_SPEED);
    }
    setModuleStates(SwerveDriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(m_chassisSpeeds));
  }

  /**
   * Set each module of the swerve drive to the corresponding desired state.
   * 
   * @param desiredStates Array of module states to set.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SEC));
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModule module = m_modules[i];
      SwerveModuleState state = desiredStates[i];
      module.setDesiredState(state, m_ignoreAngles);
    }
  }

  private void updateSmartDash() {
    // Odometry
    SmartDashboard.putNumber("Odometry: X", getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry: Y", getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry: Theta", getPoseMeters().getRotation().getDegrees());

    // Chassis Speeds
    // TODO: Measure the actual max velocity in m/s of the robot to have accurate chassis speeds.
    // SmartDashboard.putNumber("Chassis Speed: 𝚡", chassisSpeeds.vxMetersPerSecond);
    // SmartDashboard.putNumber("Chassis Speed: 𝚢", chassisSpeeds.vyMetersPerSecond);
    // SmartDashboard.putNumber("Chassis Speed: ω", chassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Gets the current chassis speeds in m/s and rad/s.
   * @return Current chassis speeds (vx, vy, ω)
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_chassisSpeeds;
  }

  /**
   * Gets the current pose of the robot.
   * 
   * @return Robot's current pose.
   */
  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  /**
   * Gets the current gyro using regression formula.
   * 
   * @return Rotation2d object holding current gyro in radians
   */
  public Rotation2d getRegGyro() {
    // * test chassis regression
    // double regCur = 0.6552670369 + m_gyro.getRotation2d().getDegrees() * 0.9926871527;
    // * new robot regression
    double regCur = 0.2507023948 + m_gyro.getRotation2d().getDegrees() * 0.999034743;
    return new Rotation2d(Math.toRadians(regCur));
  }

  /**
   * Resets the odometry of the robot to the given pose.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    Rotation2d gyroAngle = new Rotation2d(m_gyro.getRotation2d().getRadians());
    m_odometry.update(gyroAngle, m_modules[0].getState(), m_modules[1].getState(), m_modules[2].getState(), m_modules[3].getState());
  }

  /**
   * Resets pigeon.
   */
  public void resetGyro() {
    m_targetHeading = 0;
    m_gyro.reset();
  }

  /**
   * Stop all four swerve modules.
   */
  public void stopModules() {
    m_modules[0].stop();
    m_modules[1].stop();
    m_modules[2].stop();
    m_modules[3].stop();
  }

  /**
   * Sets the speed mode.
   * 
   * @param mode True if fast mode, false if slow mode.
   */
  public void setSpeedMode(boolean mode) {
    m_speedMode = mode;
  }

  public boolean getSpeedMode() {
    return m_speedMode;
  }

  public double getMaxSpeed() {
    return m_speedMode ? SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_FAST : SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  }
}
