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
  private SwerveDrive m_swerve;
  private BoomBoom m_boomBoom;
  private Turret m_turret;
  private Hood m_hood;

  // given
  private double m_gyroAngle;
  private double m_odoX;
  private double m_odoY;
  private double m_distance;

  // targets
  private double m_targetVel;
  private double m_targetHood;
  private double m_targetAngle;
  private Pose2d m_targetPoint;

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
  private boolean simMode = true;
  private DummySensor driveDummy;
  private DummySensor turretDummy;

  /**
   * Creates a new shoot command, allowing the robot to aim and be ready to fire a ball
   * TODO: Velocity Correction
   * @param sDrive Drive Train
   * @param sShooter Shooter Drum
   * @param sTurret Shooter Turret
   * @param sHood Shooter Hood
   */
  public Shoot(SwerveDrive sDrive, BoomBoom sShooter, Turret sTurret, Hood sHood) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = sDrive;
    m_boomBoom = sShooter;
    m_turret = sTurret;
    m_hood = sHood;
    
    addRequirements(m_swerve, m_boomBoom, m_turret, m_hood);

    kP = gains.kP;
    kI = gains.kI;
    kD = gains.kD;

    proportional = 0;
    integral = 0;
    derivative = 0;
    time = 0.02;

    tolerance = 5.0;
    isAimedInTolerance = false;

    if (simMode) {
      driveDummy = new DummySensor(180);
      turretDummy = new DummySensor(180);

      DummySensor.resetAll();
    }
  }

  /**
   * Updates error for custom PID.
   */
  public void updateError() {
    m_targetPoint = SwerveDriveConstants.HUB_POSE;
    m_targetAngle = AimToCenter.angleToCenter(m_odoX, m_odoY, driveDummy.get());
    // m_targetAngle = AimToCenter.angleToCenter(m_odoX, m_odoY, m_swerve.getRegGyro().getDegrees());
    error = (m_targetAngle - turretDummy.get() + 360) % 360;
    // error = (m_targetAngle - m_turret.getBoomBoomAngleDegrees() + 360) % 360;
    isAimedInTolerance = (Math.abs(error) <= tolerance);

    if (simMode) {
      SmartDashboard.putBoolean("isAimed?", isAimedInTolerance);
      System.out.println("Target Angle: " + m_targetAngle);
      System.out.println("Error: " + error);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_odoX = 0;//m_swerve.getOdometry().getX();
    m_odoY = -1;//m_swerve.getOdometry().getY();

    m_gyroAngle = m_swerve.getRegGyro().getDegrees();
    initialSwerveRotation = m_gyroAngle;

    // get targets (shooter tables)
    m_targetVel = m_boomBoom.getVelocity(m_distance);
    m_targetHood = m_boomBoom.getHood(m_distance);

    m_targetAngle = ((Math.atan2(m_odoY, m_odoX) * (180./Math.PI) - m_gyroAngle) + 180. + 360.) % 360.;

    // deadzone processing
    if (AimToCenter.isDeadzone(m_targetAngle)) {}
    
    // initial error
    updateError();
    System.out.println("Error: " + error);
    prevError = error;
  }
  /**
   * Run custom PID.
   */
  public void runPID() {
    if (error > 180){
      error = 360 - error;
      inverted = -1;
    }
    else{
      inverted = 1;
    }
    prevError = error;
    updateError();
    
    proportional = error;
    integral = integral + error * time;
    derivative = (error - prevError) / time;
    output = kP * proportional + kI * integral + kD * derivative;
    normOutput = output/360 * inverted;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (simMode) {
      System.out.println("Normalized Output: " + normOutput);
    }
      // custom pid
    runPID();

    if (simMode) {
      driveDummy.apply(normOutput);
      System.out.println("Drive Dummy: " + driveDummy.get());
    }
    m_swerve.driveWithInput(0, 0, normOutput, true); // i have no idea if this is how you rotate the
                                                 // entire swerve drive or its the line below
    // m_swerve.driveWithInput(0, 0, Math.cos(output), Math.sin(output), true);
    
    m_hood.runAngleAdjustPID(m_targetHood);
    m_boomBoom.runDrumShooterVelocityPID(m_targetVel);

    if (simMode) {
      turretDummy.apply(normOutput);
      System.out.println("Turret Dummy: " + turretDummy.get());
    }
    m_turret.m_boomBoomRotateMotor.set(normOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // return to initial swerve rotation
    m_swerve.driveWithInput(0, 0, initialSwerveRotation, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (simMode) {
    return isAimedInTolerance;
    // }
    // return false;
  }
}
