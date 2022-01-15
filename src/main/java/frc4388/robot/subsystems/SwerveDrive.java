// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;

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
  // setSwerveGains();
      
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  public SwerveModule[] modules;
  public WPI_PigeonIMU m_gyro; //TODO Add Gyro Lol

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
    // System.out.println("Inputx: " + speeds[0] + "   Inputy: " + speeds[1]);
    /*var speeds = new ChassisSpeeds(strafeX, strafeY, rotate * SwerveDriveConstants.ROTATION_SPEED //in rad/s );
    driveFromSpeeds(speeds);*/
    double xSpeedMetersPerSecond = -speeds[0] * SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND;
    double ySpeedMetersPerSecond = speeds[1] * SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND;
    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(
            fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot*3, m_gyro.getRotation2d())
              : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot*3));
      SwerveDriveKinematics.desaturateWheelSpeeds(states, Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SEC));
      for (int i = 0; i < states.length; i++) {
        SwerveModule module = modules[i];
        SwerveModuleState state = states[i];
        module.setDesiredState(state);
    }
  }
  
  @Override
  public void periodic() {
    // m_gyro.setFusedHeadingToCompass();
    // m_gyro.setYawToCompass();
    // m_gyro.getRotation2d();
    super.periodic();
  }

}