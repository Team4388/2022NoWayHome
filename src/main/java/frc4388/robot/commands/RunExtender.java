// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ExtenderConstants;
import frc4388.robot.subsystems.Extender;
import frc4388.robot.subsystems.Intake;

public class RunExtender extends CommandBase {

  private Extender extender;

  /** Creates a new RunExtender. */
  public RunExtender(Extender extender) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.extender = extender;

    addRequirements(this.extender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.extender.m_extenderMotor.set(ExtenderIntakeGroup.direction * 1.0); // TODO: change to 1.0 for actual speed, 0.5 is just for testing
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ExtenderIntakeGroup.changeDirection();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(this.extender.m_extenderMotor.getEncoder().getPosition() - ExtenderConstants.EXTENDER_FORWARD_LIMIT) < 5) {
      return true;
    }
    return false;
  }
}