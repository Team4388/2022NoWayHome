// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;

public class AimToCenter extends CommandBase {
  /** Creates a new AimWithOdometry. */
  Turret m_turret;
  SwerveDrive m_drive;

  // use odometry to find x and y later
  double x = 0;
  double y = 0;
  double z = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  double m_targetAngle;
  
  // public static Gains m_aimGains;

  public AimToCenter(Turret turret, SwerveDrive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_drive = drive;
    addRequirements(m_turret, m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (x > 0) {
      m_targetAngle = 180 + Math.atan(y/x) - m_drive.gyro.getAngle();
    } 
    if (x < 0) {
      m_targetAngle = 360 + Math.atan(y/x) - m_drive.gyro.getAngle();
    } 
    if (x == 0 && y > 0) {
      m_targetAngle = 270 - m_drive.gyro.getAngle(); 
    }
    if (x == 0 && y < 0) {
      m_targetAngle = 90 - m_drive.gyro.getAngle();
    }
    
    m_turret.runshooterRotatePID(m_targetAngle);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
