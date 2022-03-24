// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ExtenderIntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ExtenderConstants;
import frc4388.robot.subsystems.Extender;
import frc4388.robot.subsystems.Intake;

public class RunExtender extends CommandBase {

  private Extender extender;

  private double error;
  private double tolerance;

  /** Creates a new RunExtender. */
  public RunExtender(Extender extender) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extender = extender;
    
    updateError();
    tolerance = 5.0;
    
    addRequirements(this.extender);
  }
  
  public void updateError() {
    if (ExtenderIntakeGroup.direction > 0) {
      this.error = Math.abs(this.extender.getPosition() - ExtenderConstants.EXTENDER_FORWARD_LIMIT);
    } else {
      this.error = Math.abs(this.extender.getPosition() - ExtenderConstants.EXTENDER_REVERSE_LIMIT);
    }
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("RunExtender is working");
    this.extender.runExtender(ExtenderIntakeGroup.direction * 1.0);
    updateError();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ExtenderIntakeGroup.changeDirection();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (error < tolerance) {
      System.out.println("RunExtender finished");
      this.extender.runExtender(0.0);
      return true;
    }
    return false;
  }
}