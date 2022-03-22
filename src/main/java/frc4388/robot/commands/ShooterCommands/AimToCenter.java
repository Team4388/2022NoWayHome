// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.VisionOdometry;

public class AimToCenter extends CommandBase {
  /** Creates a new AimWithOdometry. */
  Turret m_turret;
  VisionOdometry m_visionOdometry;

  Supplier<Pose2d> supplier;
  Pose2d odo;

  // use odometry to find x and y later
  double x;
  double y;
  double m_targetAngle;

  // public static Gains m_aimGains;

  public AimToCenter(Turret turret, VisionOdometry visionOdometry, Supplier<Pose2d> supplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_visionOdometry = visionOdometry;

    this.supplier = supplier;

    addRequirements(m_turret, m_visionOdometry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    odo = this.supplier.get();
    // ! Yes I realize this stupid, yes it works I promise, coordinate system is funky
    x = -odo.getY();
    y = -odo.getX();
    
    SmartDashboard.putNumber("trans x", x);
    SmartDashboard.putNumber("trans y", y);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    odo = this.supplier.get(); // * update odometry using really cool supplier -aarav

    m_targetAngle = (aaravAngleToCenter(x, y, odo.getRotation().getDegrees())) % 360;
    SmartDashboard.putNumber("Target Angle", m_targetAngle);
    m_turret.runShooterRotatePID(m_targetAngle);

    // Check if limelight is within range (comment out to disable vision odo)
    if (Math.abs(m_turret.getBoomBoomAngleDegrees() - m_targetAngle) < VisionConstants.RANGE){
      // m_visionOdometry.updateOdometryWithVision();
      // m_visionOdometry.setLEDs(true);
    }
    else{
      // m_visionOdometry.setLEDs(false);
    }
  }

  public static double angleToCenter(double x, double y, double gyro) {
    double angle = ((Math.atan2(y, x) * (180./Math.PI) - gyro) + 180. + (360. * 4)) % 360.; // Finds the angle between the gyro of the robot and the target (positive x is gyro 0)
    // double angle = Math.toDegrees(Math.atan2(y, -x) - gyro);
    return (angle - 360);
  }

  public static double aaravAngleToCenter(double x, double y, double gyro) {
    
    double actualGyro = -gyro + 90;
    
    double exp = Math.toDegrees(Math.atan(y/-x)) - actualGyro;
    if (-x > 0) { return (180 + exp); }
    if (-x < 0) { return (360 + exp); }

    if (x == 0 && y > 0) { return (270 - actualGyro); } // TODO: try putting these two lines before exp is calculated
    if (x == 0 && y < 0) { return (90 - actualGyro); }

    System.err.println("Invalid case.");
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
