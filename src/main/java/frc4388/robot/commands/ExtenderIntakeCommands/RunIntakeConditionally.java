// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ExtenderIntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.subsystems.Intake;

public class RunIntakeConditionally extends CommandBase {

  private Intake intake;

  /** Creates a new RunIntakeConditionally. */
  public RunIntakeConditionally(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.intake = intake;

    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ExtenderIntakeGroup.direction > 0) {
      this.intake.runAtOutput(-1);
    } else {
      this.intake.runAtOutput(0);
    }
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
