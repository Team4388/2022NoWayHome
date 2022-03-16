// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.StorageCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Storage;
import frc4388.robot.subsystems.Turret;

public class SpitOutWrongColor extends CommandBase {

  // subsystems
  private Storage storage;
  private BoomBoom drum;
  private Turret turret;

  // time (in milliseconds)
  private long initialTime;
  private long elapsedTime;
  private long threshold;

  private double initialTurret;
  private int spitVelocity;

  /** Creates a new SpitOutWrongColor. */
  public SpitOutWrongColor(Storage storage, BoomBoom drum, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.storage = storage;
    this.drum = drum;
    this.turret = turret;

    addRequirements(this.storage, this.drum, this.turret);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = System.currentTimeMillis();
    elapsedTime = 0;
    threshold = 2000;

    initialTurret = this.turret.getEncoderPosition();
    spitVelocity = 2000;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elapsedTime = System.currentTimeMillis() - initialTime;

    this.storage.runStorage(0.9);
    this.turret.gotoMidpoint();
    this.drum.runDrumShooterVelocityPID(spitVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.storage.runStorage(0.0);
    this.turret.runShooterRotatePID(initialTurret * ShooterConstants.TURRET_DEGREES_PER_ROT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elapsedTime >= threshold);
  }
}
