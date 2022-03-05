// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.VisionOdometry;

public class TrackTarget extends CommandBase {
  /** Creates a new TrackTarget. */
  Turret m_turret;
  SwerveDrive m_drive;
  VisionOdometry m_visionOdometry;
  BoomBoom m_boomBoom;
  Hood m_hood;

  // use odometry to find x and y later
  double x;
  double y;
  double distance;
  double vel;
  double hood;
  double average;
  double output;
  Pose2d pos = new Pose2d();
  ArrayList<Point> points = new ArrayList<>();

  // public static Gains m_aimGains;

  public TrackTarget (Turret turret, BoomBoom boomBoom, Hood hood, SwerveDrive drive, VisionOdometry visionOdometry) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_drive = drive;
    m_boomBoom = boomBoom;
    m_hood = hood;
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
    //m_targetAngle = angleToCenter(x, y, m_drive.getRegGyro().getDegrees());
    m_visionOdometry.setLEDs(true);
    points = m_visionOdometry.getTargetPoints();
    double pointTotal = 0;
    for(Point point : points)
    {
      pointTotal = pointTotal + point.x;
    }
    average = pointTotal/points.size();
    output = average/VisionConstants.LIME_HIXELS * VisionConstants.TURRET_kP;
    m_turret.runTurretWithInput(output);
    try{
      pos = m_visionOdometry.getVisionOdometry();
      distance = Math.hypot(pos.getX(), pos.getY());
    }
    catch (Exception e){
    }
    vel = m_boomBoom.getVelocity(distance);
    hood = m_boomBoom.getHood(distance);
    m_boomBoom.runDrumShooterVelocityPID(vel);
    m_hood.runAngleAdjustPID(hood);
    //m_turret.runshooterRotatePID(m_targetAngle);
  }

 /* public static double angleToCenter(double x, double y, double gyro) {
    double angle = ((Math.atan2(y, x) * (180./Math.PI) - gyro) + 180. + 360.) % 360.; // Finds the angle between the gyro of the robot and the target (positive x is gyro 0)
    return angle;
  }*/

  /**
   * Checks if in hardware deadzone (due to mechanical limitations).
   * @param angle Angle to check.
   * @return True if in hardware deadzone.
   */
  public static boolean isHardwareDeadzone(double angle) {
    return ((ShooterConstants.HARD_DEADZONE_LEFT > angle) || (angle > ShooterConstants.HARD_DEADZONE_RIGHT));
  }

  /**
   * Checks if in digital deadzone (due to climber).
   * @param angle Angle to check.
   * @return True if in digital deadzone.
   */
  public static boolean isDigitalDeadzone(double angle) {
    return ((ShooterConstants.DIG_DEADZONE_LEFT < angle) && (angle < ShooterConstants.DIG_DEADZONE_RIGHT));
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
