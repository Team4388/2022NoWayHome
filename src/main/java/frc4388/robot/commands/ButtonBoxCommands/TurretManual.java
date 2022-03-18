// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ButtonBoxCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Turret;

public class TurretManual extends CommandBase {

  // subsystems
  private Turret turret;

  // booleans
  private static boolean manual = false;
  private boolean newManual = false;
  private boolean changes = false;

  /** Creates a new TurretManual. */
  public TurretManual(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;

    addRequirements(this.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    changes = (newManual != manual);

    if (manual) {
      System.out.println("Manual Turret");  // TODO: turret manual controls
    } else {
      System.out.println("Auto Turret");    // TODO: turret auto controls;
    }

    newManual = manual;
  }

  public static void setManual(boolean set)
  {
    manual = set;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manual = !manual;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return changes;
  }
}
