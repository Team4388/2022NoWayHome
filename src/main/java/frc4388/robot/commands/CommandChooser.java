// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandChooser extends CommandBase {

  private Command c1;
  private Command c2;

  private Boolean b1;
  private Boolean b2;
  
  private Command chosen;

  /** Creates a new CommandChooser. */
  public CommandChooser(Command c1, Command c2, BooleanSupplier bs1, BooleanSupplier bs2) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.c1 = c1;
    this.c2 = c2;

    this.b1 = bs1.getAsBoolean();
    this.b2 = bs2.getAsBoolean();

    Set<Subsystem> allReqs = c1.getRequirements();
    allReqs.addAll(c2.getRequirements());
    addRequirements((Subsystem[]) allReqs.toArray());
  }

  public Command getChosen() {
    if (this.b1) {
      return this.c1;
    } else {
      return this.c2;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.chosen = getChosen();
    this.chosen.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.chosen.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.chosen.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.chosen.isFinished();
  }
}
