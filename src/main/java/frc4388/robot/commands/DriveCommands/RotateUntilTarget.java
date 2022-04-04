// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.DriveCommands;

import javax.swing.plaf.basic.BasicTreeUI.TreeCancelEditingAction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.VisionOdometry;

public class RotateUntilTarget extends CommandBase {

  // subsystems
  SwerveDrive swerve;
  VisionOdometry visionOdometry;

  double rotateSpeed;

  /** Creates a new RotateUntilTarget. */
  public RotateUntilTarget(SwerveDrive swerve, VisionOdometry visionOdometry, double rotateSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerve = swerve;
    this.visionOdometry = visionOdometry;

    this.rotateSpeed = rotateSpeed;

    addRequirements(this.swerve, this.visionOdometry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.visionOdometry.setLEDs(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.swerve.driveWithInput(0.0, 0.0, rotateSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.visionOdometry.getTargetPoints() == null;
    // return this.visionOdometry.m_camera.getLatestResult().hasTargets();
  }
}
