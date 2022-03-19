// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ButtonBoxCommands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.RobotContainer;
import frc4388.robot.commands.ClimberCommands.RunClimberPath;
import frc4388.robot.commands.ShooterCommands.AimToCenter;
import frc4388.robot.subsystems.Claws;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.VisionOdometry;

public class RunTurretOrClimberAuto extends CommandBase {

  private Turret turret;
  private SwerveDrive swerveDrive;
  private VisionOdometry visionOdometry;
  private Climber climber;
  private Claws claws;

  /** Creates a new RunTurretOrClimberAuto. */
  public RunTurretOrClimberAuto(Turret turret, SwerveDrive swerveDrive, VisionOdometry visionOdometry, Climber climber, Claws claws) {
    // Use addRequirements() here to declare subsystem dependencies.
 
    this.turret = turret;
    this.swerveDrive = swerveDrive;
    this.visionOdometry = visionOdometry;
    this.climber = climber;
    this.claws = claws;

    addRequirements(this.turret, this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // check control mode here. if shooter control mode, run turret auto command. if climber control mode, run climber auto command.
    if (RobotContainer.currentControlMode.equals(RobotContainer.ControlMode.SHOOTER)) {
      new AimToCenter(this.turret, this.swerveDrive, this.visionOdometry);
    }
    if (RobotContainer.currentControlMode.equals(RobotContainer.ControlMode.CLIMBER)) {
      new RunClimberPath(this.climber, this.claws, new Point[] {new Point()});
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
