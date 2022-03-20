// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.utility.DummySensor;
import frc4388.utility.Gains;

public class Shoot extends CommandBase {

  // subsystems
  private SwerveDrive swerve;
  private Turret turret;
  private BoomBoom drum;
  private Hood hood;

  private boolean toShoot;

  // given
  private double odoX, odoY;
  private double distance;
  private double gyroAngle;

  // targets
  private double targetAngle, targetVel, targetHood;

  // pid
  private Gains gains = ShooterConstants.SHOOT_GAINS;
  private double error;
  private double prevError;
  private double kP, kI, kD;
  private double proportional, integral, derivative;
  private double output, normOutput;
  private double tolerance;

  private boolean isAimedInTolerance;
  private int inverted;
  private double initialSwerveRotation;

  /**
   * Creates a new shoot command, allowing the robot to aim and be ready to fire a ball
   * 
   * @param swerve Drive Train
   * @param drum Shooter Drum
   * @param turret Shooter Turret
   * @param hood Shooter Hood
   * 
   * @author Aarav Shah
   */
  public Shoot(SwerveDrive swerve, BoomBoom drum, Turret turret, Hood hood, boolean toShoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.drum = drum;
    this.turret = turret;
    this.hood = hood;

    this.toShoot = toShoot;

    addRequirements(this.swerve, this.drum, this.turret, this.hood);

    kP = gains.kP;
    kI = gains.kI;
    kD = gains.kD;

    proportional = 0;
    integral = 0;
    derivative = 0;

    tolerance = 10.0;
    isAimedInTolerance = false;
  }

  public Shoot(SwerveDrive swerve, BoomBoom drum, Turret turret, Hood hood) {
    this(swerve, drum, turret, hood, false);
  }

  /**
   * Updates error for custom PID.
   */
  public void updateError() {
    targetAngle = AimToCenter.aaravAngleToCenter(odoX, odoY, swerve.getRegGyro().getDegrees());
    error = (targetAngle - turret.getBoomBoomAngleDegrees() + 360) % 360;
    isAimedInTolerance = (Math.abs(error) <= tolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.turret.gotoMidpoint();

    this.odoX = 0;//-m_swerve.getOdometry().getY();
    this.odoY = -8;//-m_swerve.getOdometry().getX();

    this.distance = Math.hypot(odoX, odoY);

    this.gyroAngle = this.swerve.getRegGyro().getDegrees();
    this.initialSwerveRotation = gyroAngle;

    // get targets (shooter tables)
    this.targetVel = drum.getVelocity(distance);
    this.targetHood = drum.getHood(distance);

    this.targetAngle = AimToCenter.aaravAngleToCenter(odoX, odoY, swerve.getRegGyro().getDegrees());

    // deadzone processing
    if (AimToCenter.isDeadzone(targetAngle)) {}
    
    // initial error
    updateError();
    prevError = error;
  }

  /**
   * Run custom PID.
   */
  public void runPID() {
    if (error > 180) {
      error = 360 - error;
      this.inverted = -1;
    }
    else{
      this.inverted = 1;
    }
    prevError = error;
    updateError();
    
    this.proportional = error;
    this.integral = integral + (error * Constants.LOOP_TIME);
    this.derivative = (error - prevError) / Constants.LOOP_TIME;
    this.output = kP * proportional + kI * integral + kD * derivative;
    this.normOutput = (output / 360) * inverted;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    runPID();

    SmartDashboard.putNumber("Error", this.error);
    SmartDashboard.putNumber("Shoot.java TargetAngle", this.targetAngle);
    SmartDashboard.putNumber("Normalized Output", this.normOutput);

    this.swerve.driveWithInput(0, 0, normOutput, true);
    this.turret.m_boomBoomRotateMotor.set(normOutput);

    if (this.toShoot) {
      this.hood.runAngleAdjustPID(this.targetHood);
      this.drum.runDrumShooterVelocityPID(this.targetVel);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // return to initial swerve rotation
    // swerve.driveWithInput(0, 0, initialSwerveRotation, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("isAimedInTolerance", isAimedInTolerance);
    return isAimedInTolerance;
  }
}
