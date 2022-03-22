// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RunCommandForTime extends CommandBase {

  // command
  Command command;

  // timing
  long start;
  long elapsed;
  double duration;

  // override
  boolean override;

  /**
   * Runs given command for given time.
   * @param command Command to run.
   * @param duration Time to run for, in seconds.
   * @param override If true: end command when time ends, even if the command isn't finished. If false: end command when it finished and time has ended.
   */
  public RunCommandForTime(Command command, double duration, boolean override) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.command = command;
    this.duration = duration * 1000; // ! convert seconds to milliseconds, duh
    this.override = override;

    addRequirements(this.command.getRequirements().toArray(Subsystem[]::new));
  }

  /**
   * Runs given command for given time.
   * @param command Command to run.
   * @param duration Time to run for, in seconds.
   */
  public RunCommandForTime(Command command, double duration) {
    this(command, duration, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.start = System.currentTimeMillis();
    this.command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.elapsed = System.currentTimeMillis() - this.start;
    this.command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.override) {
      return (this.elapsed >= this.duration);
    } else {
      return (this.command.isFinished() && (this.elapsed >= this.duration));
    }
  }
}
