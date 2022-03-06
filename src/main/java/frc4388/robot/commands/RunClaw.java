// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ClawConstants;
import frc4388.robot.subsystems.Claws;

public class RunClaw extends CommandBase {

  // parameters
  public Claws m_claws;
  public Claws.ClawType clawType;
  public boolean open;

  /**
   * Creates a new RunClaw, which runs a claw.
   * @param sClaws Claws subsystem.
   * @param which Which claw to run.
   * @param open Whether to open or close the claw.
   */
  public RunClaw(Claws sClaws, Claws.ClawType which, boolean open) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_claws = sClaws;
    clawType = which;
    this.open = open;

    addRequirements(m_claws);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_claws.runClaw(clawType, open);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_claws.checkSwitchAndCurrent(clawType);
  }
}
