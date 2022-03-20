// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  private BoomBoom drum;
  private Turret turret;
  private Hood hood;

  private boolean toShoot;

  // given
  private double gyroAngle;
  private double odoX;
  private double odoY;
  private double distance;

  // targets
  private double targetVel;
  private double targetHood;
  private double targetAngle;
  private Pose2d targetPoint;

  // pid
  private double error;
  private double prevError;
  private Gains gains = ShooterConstants.SHOOT_GAINS;
  private double kP, kI, kD;
  private double proportional, integral, derivative;
  private double time;
  private double output;
  private double normOutput;
  private double tolerance;

  private boolean isAimedInTolerance;
  private int inverted;
  private double initialSwerveRotation;

  // testing
  private boolean simMode = false;
  private DummySensor driveDummy;
  private DummySensor turretDummy;

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
    time = 0.02;

    tolerance = 10.0;
    isAimedInTolerance = false;

    if (simMode) {
      driveDummy = new DummySensor(180);
      turretDummy = new DummySensor(180);

      DummySensor.resetAll();
    }
  }

  public Shoot(SwerveDrive swerve, BoomBoom drum, Turret turret, Hood hood) {
    this(swerve, drum, turret, hood, false);
  }

  /**
   * Updates error for custom PID.
   */
  public void updateError() {
    targetPoint = SwerveDriveConstants.HUB_POSE;
    // m_targetAngle = AimToCenter.angleToCenter(m_odoX, m_odoY, driveDummy.get());
    targetAngle = AimToCenter.aaravAngleToCenter(odoX, odoY, swerve.getRegGyro().getDegrees());
    // error = (m_targetAngle - turretDummy.get() + 360) % 360;
    error = (targetAngle - turret.getBoomBoomAngleDegrees() + 360) % 360;
    isAimedInTolerance = (Math.abs(error) <= tolerance);

    if (simMode) {
      SmartDashboard.putBoolean("isAimed?", isAimedInTolerance);
      System.out.println("Target Angle: " + targetAngle);
      System.out.println("Error: " + error);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    turret.gotoMidpoint();

    odoX = 0;//-m_swerve.getOdometry().getY();
    odoY = -8;//-m_swerve.getOdometry().getX();

    gyroAngle = swerve.getRegGyro().getDegrees();
    initialSwerveRotation = gyroAngle;

    // get targets (shooter tables)
    targetVel = drum.getVelocity(distance);
    targetHood = drum.getHood(distance);

    targetAngle = AimToCenter.aaravAngleToCenter(odoX, odoY, swerve.getRegGyro().getDegrees());

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
      inverted = -1;
    }
    else{
      inverted = 1;
    }
    prevError = error;
    updateError();
    
    proportional = error;
    integral = integral + (error * time);
    derivative = (error - prevError) / time;
    output = kP * proportional + kI * integral + kD * derivative;
    normOutput = (output / 360) * inverted;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if (simMode) {
      System.out.println("Normalized Output: " + normOutput);
    }
    // custom pid
    
    if (simMode) {
      driveDummy.apply(normOutput);
      System.out.println("Drive Dummy: " + driveDummy.get());
    }

    runPID();

    SmartDashboard.putNumber("Error", this.error);
    SmartDashboard.putNumber("Shoot.java TargetAngle", this.targetAngle);
    SmartDashboard.putNumber("Normalized Output", normOutput);

    swerve.driveWithInput(0, 0, normOutput, true);
    turret.m_boomBoomRotateMotor.set(normOutput);

    if (this.toShoot) {
      this.hood.runAngleAdjustPID(this.targetHood);
      this.drum.runDrumShooterVelocityPID(this.targetVel);
    }

    if (simMode) {
      turretDummy.apply(normOutput);
      System.out.println("Turret Dummy: " + turretDummy.get());
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
    // if (simMode) {
    SmartDashboard.putBoolean("isAimedInTolerance", isAimedInTolerance);
    return isAimedInTolerance;
    // }
    // return false;
  }
}
