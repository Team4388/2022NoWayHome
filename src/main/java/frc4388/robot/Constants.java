// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc4388.utility.Gains;
import frc4388.utility.LEDPatterns;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveDriveConstants {
    public static final double ROTATION_SPEED = 4;
    public static final double WIDTH = 23.5;
    public static final double HEIGHT = 25.5;
    public static final double JOYSTICK_TO_METERS_PER_SECOND_FAST = 11;
    public static final double JOYSTICK_TO_METERS_PER_SECOND_SLOW = 2;
    public static final double MAX_SPEED_FEET_PER_SEC = 20; // TODO: redundant constant?
    public static final double SPEED_FEET_PER_SECOND_AT_FULL_POWER = 20; // TODO: redundant constant?

    // IDs
    public static final int LEFT_FRONT_STEER_CAN_ID = 2;
    public static final int LEFT_FRONT_WHEEL_CAN_ID = 3;
    public static final int RIGHT_FRONT_STEER_CAN_ID = 4;
    public static final int RIGHT_FRONT_WHEEL_CAN_ID = 5;
    public static final int LEFT_BACK_STEER_CAN_ID = 6;
    public static final int LEFT_BACK_WHEEL_CAN_ID = 7;
    public static final int RIGHT_BACK_STEER_CAN_ID = 8;
    public static final int RIGHT_BACK_WHEEL_CAN_ID = 9;
    public static final int LEFT_FRONT_STEER_CAN_ENCODER_ID = 10;
    public static final int RIGHT_FRONT_STEER_CAN_ENCODER_ID = 11;
    public static final int LEFT_BACK_STEER_CAN_ENCODER_ID = 12;
    public static final int RIGHT_BACK_STEER_CAN_ENCODER_ID = 13;
    public static final int GYRO_ID = 14;

    // offsets are in degrees
    // NATHAN if you truncate or round or simplify these i will cry
    // public static final double LEFT_FRONT_ENCODER_OFFSET = 181.45 - 3.30;//
    // 181.7578125;//180.0;//315.0 +45;//180.0;
    // public static final double RIGHT_FRONT_ENCODER_OFFSET = 360. - 59.0625 +
    // 0.18;// 360.-59.0625;//315.0;//224.296875 +
    // public static final double LEFT_BACK_ENCODER_OFFSET = 360. - 128.222;//
    // 308.408203125;//225.0;//45.87890625;//360.0
    // public static final double RIGHT_BACK_ENCODER_OFFSET = 360. + 2.15 - 3.637;//
    // 180-2.021484375;//0.0;//134.384765625

    public static final double RIGHT_FRONT_ENCODER_OFFSET = (4 * 360. - 152.05 - 180 - 90) % 360.;
    public static final double LEFT_FRONT_ENCODER_OFFSET = (4 * 360. - 232.58 + 180 - 90 ) % 360.;
    public static final double LEFT_BACK_ENCODER_OFFSET = (4 * 360. - 189.50 - 90) % 360.;
    public static final double RIGHT_BACK_ENCODER_OFFSET = (4 * 360. - 9.31 - 90 - 180) % 360.;

    // swerve PID constants
    public static final int SWERVE_SLOT_IDX = 0;
    public static final int SWERVE_PID_LOOP_IDX = 1;
    public static final int SWERVE_TIMEOUT_MS = 30;
    public static final Gains SWERVE_GAINS = new Gains(1.5, 0.0, 0.0, 0.0, 0, 1.0);

    // swerve auto constants
    public static final PIDController X_CONTROLLER = new PIDController(0.5, 0.0, 0.0);
    public static final PIDController Y_CONTROLLER = new PIDController(2.0, 0.0, 0.0);
    public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(15.0, 0.1, 0.3,
        new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    public static final boolean PATH_RECORD_VELOCITY = true;
    public static final double PATH_MAX_VELOCITY = 5.0;
    public static final double PATH_MAX_ACCELERATION = 5.0;
    public static final double MIN_WAYPOINT_ANGLE = 20;
    public static final double MIN_WAYPOINT_DISTANCE = 0.1;
    public static final double MIN_WAYPOINT_VELOCITY = 0.1;

    // swerve configuration
    public static final double NEUTRAL_DEADBAND = 0.04;
    public static final double OPEN_LOOP_RAMP_RATE = 0.2;
    public static final int REMOTE_0 = 0;

    // conversions
    // gear ratio: 5.14 rev motor = 1 rev wheel
    // wheel diameter: official = 4 in, measured = 3.8 in
    /* Ratio Calculation */
    public static final double MOTOR_REV_PER_STEER_REV = 12.8;
    public static final double MOTOR_REV_PER_WHEEL_REV = 6.12;// 5.142857;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double TICKS_PER_MOTOR_REV = 2048;
    public static final double INCHES_PER_WHEEL_REV = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double INCHES_PER_METER = 39.370;
    public static final double METERS_PER_INCH = 1 / INCHES_PER_METER;

    public static final double WHEEL_REV_PER_MOTOR_REV = 1 / MOTOR_REV_PER_WHEEL_REV;
    public static final double TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * MOTOR_REV_PER_WHEEL_REV;
    public static final double TICKS_PER_INCH = TICKS_PER_WHEEL_REV / INCHES_PER_WHEEL_REV;
    public static final double INCHES_PER_TICK = 1 / TICKS_PER_INCH;
    public static final double TICK_TIME_TO_SECONDS = 0.1;
    public static final double SECONDS_TO_TICK_TIME = 1 / TICK_TIME_TO_SECONDS;

    // misc
    public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
    public static final Pose2d HUB_POSE = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
  }

  public static final class SerializerConstants {
    public static final double SERIALIZER_BELT_SPEED = 0.1d;
    
    // CAN IDs
    public static final int SERIALIZER_BELT = 17;
    public static final int SERIALIZER_BELT_BEAM = 27; // TODO
  }
  
  public static final class IntakeConstants {
    // CAN IDs
    public static final int INTAKE_MOTOR = 15;
    public static final int EXTENDER_MOTOR = 16;
  }
  public static final class StorageConstants {
    public static final int STORAGE_CAN_ID = 18;
    public static final int BEAM_SENSOR_SHOOTER = 28; //TODO
    public static final int BEAM_SENSOR_INTAKE = 29; //TODO
    public static final double STORAGE_SPEED = 0.3;
  }
  public static final class LEDConstants {
    public static final int LED_SPARK_ID = 0;

    public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;
  }

  /**
   * The OIConstants class contains the ID for the XBox controllers
   */
  public static final class OIConstants {
    public static final int XBOX_DRIVER_ID = 0;
    public static final int XBOX_OPERATOR_ID = 1;
    public static final double LEFT_AXIS_DEADBAND = 0.1;
    public static final double RIGHT_AXIS_DEADBAND = 0.6;
  }

  public static final class ShooterConstants {
    /* PID Constants Shooter */
    public static final int CLOSED_LOOP_TIME_MS = 1;

    public static final int SHOOTER_TIMEOUT_MS = 32;
    public static final int SHOOTER_SLOT_IDX = 0;
    public static final int SHOOTER_PID_LOOP_IDX = 1;
    public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIG = new SupplyCurrentLimitConfiguration(
        true, 60, 40, 0.5);
    public static final int SHOOTER_FALCON_LEFT_CAN_ID = 21;
    public static final int SHOOTER_FALCON_RIGHT_CAN_ID = 22;
    public static final double TURRET_SPEED_MULTIPLIER = 0.4d;
    public static final int DEGREES_PER_ROT = 0;
    public static final int TURRET_MOTOR_POS_AT_ZERO_ROT = 0;
    public static final int TURRET_MOTOR_ROTS_PER_ROT = 0;
    public static final double ENCODER_TICKS_PER_REV = 2048;

    // Shoot Command Constants
    public static final Gains SHOOT_GAINS = new Gains(5.0, 0.0, 0.0, 0.0, 0, 1.0);

    /* Turret Constants */
    // ID
    public static final int TURRET_MOTOR_CAN_ID = 19;
    public static final Gains SHOOTER_TURRET_GAINS = new Gains(0.6, 0.0, 0.0, 0.0, 0, 1.0);
    public static final Gains SHOOTER_ANGLE_GAINS = new Gains(0.05, 0.0, 0.0, 0.0, 0, 0.3);
    public static final double SHOOTER_TURRET_MIN = -1.0;
    public static final double TURRET_FORWARD_LIMIT = 61.7; // TODO: find
    public static final double TURRET_REVERSE_LIMIT = -42.3; // TODO: find

    public static final Gains DRUM_SHOOTER_GAINS = new Gains(0, 0, 0, 0, 0, 0); // TODO: tune values

    /* Hood Constants */
    public static final int SHOOTER_ANGLE_ADJUST_ID = 20;
    public static final double HOOD_MOTOR_ROTS_PER_ROT = 1; // TODO: Find
    public static final double HOOD_MOTOR_POS_AT_ZERO_ROT = 0; // TODO: Find
    public static final double HOOD_FORWARD_LIMIT = 48.69; // TODO: find
    public static final double HOOD_REVERSE_LIMIT = -100; // TODO: find

  }

  public static final class VisionConstants {
    // public static final double TURN_P_VALUE = 0.8;
    // public static final double X_ANGLE_ERROR = 0.5;
    // public static final double GRAV = 385.83;
    // public static final double TARGET_HEIGHT = 67.5;
    // public static final double FOV = 29.8; //Field of view limelight

    public static final double LIME_ANGLE = 24.7;

    public static final String NAME = "photonCamera";

    public static final double TARGET_HEIGHT = 8*12 + 8; //TODO: Convert to metric (does this still need to be converted?)
    public static final double TARGET_RADIUS = 4*12; //TODO: Convert to metric (does this still need to be converted?)
    public static final double H_FOV = 59.6;
    public static final double V_FOV = 49.7;
    public static final double LIME_VIXELS = 960;
    public static final double LIME_HIXELS = 720;
    public static final double TURRET_kP = 0.1;

    public static final double RANGE = 10;

    public static final double LIMELIGHT_RADIUS = 1.d;
    public static final double SHOOTER_CORRECTION = 1.d;
  }
}
