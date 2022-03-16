// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.climber;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Claws;
import frc4388.robot.subsystems.Climber;

public class RunClimberPath extends CommandBase {
  Climber climber;
  Claws claws;

  Point[] path;
  int nextIndex;

  /** Creates a new RunClimberPath. */
  public RunClimberPath(Point[] _path, Climber _climber, Claws _claws) {
    path = _path;

    climber = _climber;
    claws = _claws;
    addRequirements(climber, claws);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claws.setOpen(true);
    nextIndex = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!claws.fullyOpen())
      return;
    
    Vector2d vec = new Vector2d(1,1);
    double mag = vec.magnitude();
    vec.x /= mag;
    vec.y /= mag;

    climber.controlWithInput(vec.x * .02, vec.y * .02);
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
