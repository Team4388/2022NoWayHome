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
import frc4388.utility.Gains;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;

  public SwerveModule[] modules;
  public WPI_Pigeon2 m_gyro;

  public SwerveDriveOdometry m_odometry;
  // public SwerveDriveOdometry m_odometry;

  public double speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  public boolean ignoreAngles;
  public double rotTarget;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  public final Field2d m_field = new Field2d();

  public SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, WPI_Pigeon2 gyro) {
    m_frontLeft = frontLeft;
    m_frontRight = frontRight;
    m_backLeft = backLeft;
    m_backRight = backRight;
    m_gyro = gyro;

    modules = new SwerveModule[] { m_frontLeft, m_frontRight, m_backLeft, m_backRight };
    SmartDashboard.putNumber("OMEGA", 0.0);
    // m_poseEstimator = new SwerveDrivePoseEstimator(
    // getRegGyro(),//m_gyro.getRotation2d(),
    // new Pose2d(),
    // m_kinematics,
    // VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(1)), // TODO: tune
    // VecBuilder.fill(Units.degreesToRadians(1)), // TODO: tune
    // VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(1))); // TODO: tune

    m_odometry = new SwerveDriveOdometry(SwerveDriveConstants.DRIVE_KINEMATICS, m_gyro.getRotation2d());

    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);
  }

  public void driveWithInput(double speedX, double speedY, double rot, boolean fieldRelative) {
    Translation2d speed = new Translation2d(speedX, speedY);
    driveWithInput(speed, rot, fieldRelative);
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
    ignoreAngles = (speed.getX() == 0) && (speed.getY() == 0) && (rot == 0);

    double mag = speed.getNorm();
    speed = speed.times(mag * speedAdjust);

    double xSpeedMetersPerSecond = speed.getX();
    double ySpeedMetersPerSecond = speed.getY();
    chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, -rot * SwerveDriveConstants.ROTATION_SPEED * 2, new Rotation2d(-m_gyro.getRotation2d().getRadians() + (Math.PI * 2) + (Math.PI / 2))) : new ChassisSpeeds(ySpeedMetersPerSecond, -xSpeedMetersPerSecond, -rot * SwerveDriveConstants.ROTATION_SPEED * 2);
    SwerveModuleState[] states = SwerveDriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {
    if (fieldRelative) {
      if (rightStick.getNorm() > OIConstants.RIGHT_AXIS_DEADBAND) rotTarget = -Math.atan2(rightStick.getY(), rightStick.getX());
      double rotDifference = rotTarget - m_gyro.getRotation2d().getRadians();
      leftStick = leftStick.times(leftStick.getNorm() * speedAdjust);
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(leftStick.getX(), leftStick.getY(), rotDifference, new Rotation2d(-m_gyro.getRotation2d().getRadians() + (Math.PI * 2) + (Math.PI / 2)));
    } else chassisSpeeds = new ChassisSpeeds(leftStick.getX(), leftStick.getY(), rightStick.getX() * SwerveDriveConstants.ROTATION_SPEED * 2);
    setModuleStates(SwerveDriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
  }

  /**
   * Set each module of the swerve drive to the corresponding desired state.
   * 
   * @param desiredStates Array of module states to set.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SEC));
    // int i = 2; {
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = desiredStates[i];
      module.setDesiredState(state, ignoreAngles);
    }
    // modules[0].setDesiredState(desiredStates[0], false);
  }

  public void setModuleRotationsToAngle(double angle) {
    for (int i = 0; i < modules.length; i++) {
      SwerveModule module = modules[i];
      module.rotateToAngle(angle);
    }
  }

  @Override
  public void periodic() {

    updateOdometry();
    updateSmartDash();

    // SmartDashboard.putNumber("Pigeon getRotation2d", m_gyro.getRotation2d().getDegrees());
    // SmartDashboard.putNumber("Pigeon getAngle", m_gyro.getAngle());
    // SmartDashboard.putNumber("Pigeon Yaw", m_gyro.getYaw());
    // SmartDashboard.putNumber("Pigeon Yaw (0 to 360)", m_gyro.getYaw() % 360);

    m_field.setRobotPose(getOdometry());
    super.periodic();
  }

  private void updateSmartDash() {
    // odometry
    SmartDashboard.putNumber("Odometry: X", getOdometry().getX());
    SmartDashboard.putNumber("Odometry: Y", getOdometry().getY());
    SmartDashboard.putNumber("Odometry: Theta", getOdometry().getRotation().getDegrees());

    // chassis speeds
    // TODO: find the actual max velocity in m/s of the robot in fast mode to have accurate chassis speeds
    // SmartDashboard.putNumber("Chassis Vel: X", chassisSpeeds.vxMetersPerSecond);
    // SmartDashboard.putNumber("Chassis Vel: Y", chassisSpeeds.vyMetersPerSecond);
    // SmartDashboard.putNumber("Chassis Vel: ω", chassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Gets the current chassis speeds in m/s and rad/s.
   * @return Current chassis speeds (vx, vy, ω)
   */
  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  /**
   * Gets the current pose of the robot.
   * 
   * @return Robot's current pose.
   */
  public Pose2d getOdometry() {
    // return m_odometry.getPoseMeters();
    return m_odometry.getPoseMeters();
    // return m_poseEstimator.getEstimatedPosition();
  }

  public Pose2d getAutoOdo() {
    Pose2d workingPose = getOdometry();
    return new Pose2d(-workingPose.getX(), workingPose.getY(), workingPose.getRotation());
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
    Rotation2d actualDWI = new Rotation2d(-m_gyro.getRotation2d().getRadians() + (Math.PI * 2)); // + (Math.PI/2));
    Rotation2d actual = new Rotation2d(m_gyro.getRotation2d().getRadians());

    SmartDashboard.putNumber("AUTO ACTUAL GYRO", actual.getDegrees());
    SmartDashboard.putNumber("AUTO DWI GYRO", actual.getDegrees());

    m_odometry.update(actual, // m_gyro.getRotation2d(),//new Rotation2d((2 * Math.PI) - getRegGyro().getRadians()),
        modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
  }

  /**
   * Resets pigeon.
   */
  public void resetGyro() {
    m_gyro.reset();
    rotTarget = 0;
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

  public double getCurrent() {
    return m_frontLeft.getCurrent() + m_frontRight.getCurrent() + m_backRight.getCurrent() + m_backLeft.getCurrent();
  }

  public double getVoltage() {
    return m_frontLeft.getVoltage() + m_frontRight.getVoltage() + m_backRight.getVoltage() + m_backLeft.getVoltage();
  }
}
