// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.nio.file.SecureDirectoryStream;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.SwerveDrive;

public class Shoot extends CommandBase {

  public SwerveDrive m_swerve;
  public BoomBoom m_boomBoom;
  public Hood m_hood;

  public double m_targetVel;
  public double m_targetHood;
  public double m_distance;

  /** Creates a new Shoot. */
  public Shoot(SwerveDrive sDrive, BoomBoom sShooter, Hood sHood) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = sDrive;
    m_boomBoom = sShooter;
    m_hood = sHood;
    
    addRequirements(m_swerve, m_boomBoom, m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_distance = 0; //TODO: get this value using odometry
    m_targetVel = m_boomBoom.getVelocity(m_distance);
    m_targetHood = m_boomBoom.getHood(m_distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hood.runAngleAdjustPID(m_targetHood);
    m_boomBoom.runDrumShooterVelocityPID(m_targetVel);
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
