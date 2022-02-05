// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public static final double WHEEL_SPEED = 0.1;
    public static final double WIDTH = 15.25;
    public static final double HEIGHT = 15.25;
    public static final double JOYSTICK_TO_METERS_PER_SECOND_FAST = 11;
    public static final double JOYSTICK_TO_METERS_PER_SECOND_SLOW = 2;
    public static final double MAX_SPEED_FEET_PER_SEC = 20; // redundant constant?
    public static final double SPEED_FEET_PER_SECOND_AT_FULL_POWER = 20; // redundant constant?
    
    //IDs
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
    
    // ofsets are in degrees
    public static final double LEFT_FRONT_ENCODER_OFFSET = 180.0;
    public static final double RIGHT_FRONT_ENCODER_OFFSET = 300.0;
    public static final double LEFT_BACK_ENCODER_OFFSET = 360.0 - 128.0;
    public static final double RIGHT_BACK_ENCODER_OFFSET = 0.0;

    // swerve PID constants
    public static final int SWERVE_SLOT_IDX = 0;
    public static final int SWERVE_PID_LOOP_IDX = 1;
    public static final int SWERVE_TIMEOUT_MS = 30;
    public static final Gains SWERVE_GAINS = new Gains(1.0, 0.0, 1.0, 0.0, 0, 1.0);

    // swerve auto constants
    public static final PIDController X_CONTROLLER = new PIDController(0.0, 0.0, 0.0);
    public static final PIDController Y_CONTROLLER = new PIDController(0.0, 0.0, 0.0);
    public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(
      1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));

    // swerve configuration
    public static final double NEUTRAL_DEADBAND = 0.04;
    public static final double OPEN_LOOP_RAMP_RATE = 0.2;
    public static final int REMOTE_0 = 0;

    // conversions
    // gear ratio: 5.12 rev motor = 1 rev wheel
    // wheel diameter: official = 4 in, measured = 3.8 in
    /* Ratio Calculation */
    public static final double MOTOR_REV_PER_WHEEL_REV = 5.12;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double TICKS_PER_MOTOR_REV = 2048;
    public static final double INCHES_PER_WHEEL_REV = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double INCHES_PER_METER = 39.370;
    public static final double METERS_PER_INCH = 1/INCHES_PER_METER;
    
    public static final double WHEEL_REV_PER_MOTOR_REV = 1/MOTOR_REV_PER_WHEEL_REV;
    public static final double TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * MOTOR_REV_PER_WHEEL_REV;
    public static final double TICKS_PER_INCH = TICKS_PER_WHEEL_REV/INCHES_PER_WHEEL_REV;
    public static final double INCHES_PER_TICK = 1/TICKS_PER_INCH;
    public static final double TICK_TIME_TO_SECONDS = 0.1;
    public static final double SECONDS_TO_TICK_TIME = 1/TICK_TIME_TO_SECONDS;

    // misc
    public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
    // TODO: put in real numbers for the hub
    public static final Pose2d HUB_POSE = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
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
}
