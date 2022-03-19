// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandChooser extends CommandBase {

  private Command c1;
  private Command c2;

  private BooleanSupplier bs1;
  private BooleanSupplier bs2;

  /** Creates a new CommandChooser. */
  public CommandChooser(Command c1, Command c2, BooleanSupplier bs1, BooleanSupplier bs2) {
    // Use addRequirements() here to declare subsystem dependencies.


    addRequirements(c1.getRequirements().toArray());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
