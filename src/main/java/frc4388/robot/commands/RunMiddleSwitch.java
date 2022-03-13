// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunMiddleSwitch extends CommandBase {
  
  private static boolean manual = false;
  private boolean newManual = false;
  private boolean changes = false;

  /** Creates a new RunMiddleSwitch. */
  public RunMiddleSwitch() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    changes = (newManual == manual);

    if (manual) {
      printManual();
    } else {
      printNormal();
    }

    newManual = manual;
  }

  public void printNormal(){
    System.out.println("Normal");
  }

  public void printManual(){
    System.out.println("Manual");
  }

  public static void setManual(boolean set)
  {
    manual = set;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return changes;
  }
}
