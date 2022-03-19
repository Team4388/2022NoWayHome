// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.util.Collections;
import java.util.HashMap;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandChooser extends CommandBase {

  private HashMap<Command, BooleanSupplier> commandMap;
  
  // // private Command chosen;

  /** Creates a new CommandChooser. */
  public CommandChooser(HashMap<Command, BooleanSupplier> commandMap) {
    this.commandMap = commandMap;

    Set<Subsystem> allReqs = Collections.emptySet();

    for(Command command : commandMap.keySet())
      allReqs.addAll(command.getRequirements());

    addRequirements((Subsystem[]) allReqs.toArray());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for(Command command : commandMap.keySet())
      if(commandMap.get(command).getAsBoolean()) command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for(Command command : commandMap.keySet())
      if(commandMap.get(command).getAsBoolean()) command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for(Command command : commandMap.keySet())
      if(commandMap.get(command).getAsBoolean()) command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = true;

    for(Command command : commandMap.keySet())
      if(commandMap.get(command).getAsBoolean()) finished &= command.isFinished();
    
    return finished;
  }
}
