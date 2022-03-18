// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.RobotContainer;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.Turret;

public class RunTurretOrClimber extends CommandBase {

  Turret turret;
  Climber climber;

  /** Creates a new RunTurretOrClimber. */
  public RunTurretOrClimber(Turret turret, Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.turret = turret;
    this.climber = climber;

    addRequirements(this.turret, this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (RobotContainer.currentMode.equals(RobotContainer.Mode.TURRET)) {
    //   this.turret.runTurretWithInput(getOperatorController().getLeftX())
    // } else if (RobotContainer.currentMode.equals(RobotContainer.Mode.CLIMBER)) {

    // }
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
