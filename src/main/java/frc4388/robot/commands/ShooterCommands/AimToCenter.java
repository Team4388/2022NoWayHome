// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.VisionOdometry;

public class AimToCenter extends CommandBase {
  /** Creates a new AimWithOdometry. */
  Turret m_turret;
  SwerveDrive m_drive;
  VisionOdometry m_visionOdometry;

  // use odometry to find x and y later
  double x;
  double y;
  double m_targetAngle;

  // public static Gains m_aimGains;

  public AimToCenter(Turret turret, SwerveDrive drive, VisionOdometry visionOdometry) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_drive = drive;
    m_visionOdometry = visionOdometry;
    addRequirements(m_turret, m_drive, m_visionOdometry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = 0;
    y = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_targetAngle = angleToCenter(x, y, m_drive.getRegGyro().getDegrees());
    m_turret.runShooterRotatePID(m_targetAngle);

    // Check if limelight is within range (comment out to disable vision odo)
    if (Math.abs(m_turret.getBoomBoomAngleDegrees() - m_targetAngle) < VisionConstants.RANGE){
      m_visionOdometry.updateOdometryWithVision();
      m_visionOdometry.setLEDs(true);
    }
    else{
      m_visionOdometry.setLEDs(false);
    }
  }

  public static double angleToCenter(double x, double y, double gyro) {
    double angle = ((Math.atan2(y, x) * (180./Math.PI) - gyro) + 180. + 360.) % 360.; // Finds the angle between the gyro of the robot and the target (positive x is gyro 0)
    return angle;
  }

  public static double aaravAngleToCenter(double x, double y, double gyro) {
    double exp = Math.toDegrees(Math.atan(y/x)) - gyro;
    if (x > 0) { return exp; }
    if (x < 0) { return (180 + exp); }

    if (x == 0 && y > 0) { return (90 - gyro); }
    if (x == 0 && y < 0) { return (-1 * gyro); }

    System.out.println("Invalid case.");
    return 0;
  }

  /**
   * Checks if in deadzone.
   * @param angle Angle to check.
   * @return True if in deadzone.
   */
  public static boolean isDeadzone(double angle) {
    if (angle == Double.NaN) {
      return false;
    }
    return !((ShooterConstants.TURRET_REVERSE_SOFT_LIMIT <= angle) && (angle <= ShooterConstants.TURRET_FORWARD_SOFT_LIMIT));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
