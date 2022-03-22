// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;

public class DriveWithInputForTime extends CommandBase {

  // subsystems
  SwerveDrive swerve;

  double[] inputs;

  // timing
  long start;
  long elapsed;
  double duration;

  /**
   * DriveWithInput for a specified amount of time.
   * @param inputs Inputs used in DriveWithInput (xspeed, yspeed, xrot, yrot).
   * @param time Time to DriveWithInput for, in seconds.
   */
  public DriveWithInputForTime(SwerveDrive swerve, double[] inputs, double duration) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerve = swerve;
    this.inputs = inputs;
    this.duration = duration * 1000; // ! convert seconds to milliseconds, duh

    addRequirements(this.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("RUNNING");
    elapsed = System.currentTimeMillis() - start;
    this.swerve.driveWithInput(inputs[0], inputs[1], inputs[2], inputs[3], true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Duration: " + duration);
    System.out.println("Elapsed: " + elapsed);

    return ((double) elapsed >= duration);
  }
}
