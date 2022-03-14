// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Extender;
import frc4388.robot.subsystems.Intake;

public class RunExtenderOut extends CommandBase {

  private Intake intake;
  private Extender extender;
  private int direction;

  /** Creates a new RunExtenderOut. */
  public RunExtenderOut(Intake intake, Extender extender) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    this.extender = extender;

    this.direction = 1;

    addRequirements(this.intake, this.extender);
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
