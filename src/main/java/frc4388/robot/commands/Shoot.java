// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.hal.simulation.SimulatorJNI;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.utility.DummySensor;
import frc4388.utility.Gains;

public class Shoot extends CommandBase {

  // subsystems
  public SwerveDrive m_swerve;
  public BoomBoom m_boomBoom;
  public Turret m_turret;
  public Hood m_hood;

  // given
  public double m_gyroAngle;
  public double m_odoX;
  public double m_odoY;
  public double m_distance;

  // targets
  public double m_targetVel;
  public double m_targetHood;
  public double m_targetAngle;
  public double m_driveTargetAngle;

  // pid
  public double error;
  public double prevError;
  public Gains shootGains = ShooterConstants.SHOOT_GAINS;
  public double kP, kI, kD;
  public double proportional, integral, derivative;
  public double time;
  public double output;
  public double tolerance = 5.0;

  // testing
  public DummySensor dummy = new DummySensor(0);

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

    kP = shootGains.kP;
    kI = shootGains.kI;
    kD = shootGains.kD;

    proportional = 0;
    integral = 0;
    derivative = 0;
    time = 0.02;

    DummySensor.resetAll();
  }

  /**
   * Updates error for custom PID.
   */
  public void updateError() {
    error = (m_targetAngle - m_turret.getBoomBoomAngleDegrees() + 360) % 360;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_odoX = m_swerve.getOdometry().getX();
    m_odoY = m_swerve.getOdometry().getY();
    m_distance = Math.sqrt(Math.pow(m_odoX, 2) + Math.pow(m_odoY, 2));

    m_gyroAngle = m_swerve.getRegGyro().getDegrees();

    // get targets (shooter tables)
    m_targetVel = m_boomBoom.getVelocity(m_distance);
    m_targetHood = m_boomBoom.getHood(m_distance);

    // target angle tests
    m_gyroAngle = 0;
    m_odoX = -1;
    m_odoY = 1;

    m_targetAngle = ((Math.atan2(m_odoY, m_odoX) * (180./Math.PI) - m_gyroAngle) + 180. + 360.) % 360.;

    // deadzone processing
    if (AimToCenter.isHardwareDeadzone(m_targetAngle)) {
      m_targetAngle = m_targetAngle + 20;
    }

    if (AimToCenter.isDigitalDeadzone(m_targetAngle)) {
      // this should rotate the entire swerve drive by 20 degrees, so shoot can now proceed like normal. idk if this will work
      m_swerve.driveWithInput(0, 0, Math.cos(m_gyroAngle + 20), Math.sin(m_gyroAngle + 20), true);
    }
    
    // initial error
    updateError();
    System.out.println("Error: " + error);
    prevError = error;
  }
  /**
   * Run custom PID.
   */
  public void runPID() {
    prevError = error;
    updateError();
    
    proportional = error;
    integral = integral + error * time;
    derivative = (error - prevError) / time;
    output = kP * proportional + kI * integral + kD * derivative;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // custom pid
    runPID();
    // m_swerve.driveWithInput(0, 0, output, true); // i have no idea if this is how you rotate the
                                                 // entire swerve drive or its the line below
    m_swerve.driveWithInput(0, 0, Math.cos(output), Math.sin(output), true);
    
    m_hood.runAngleAdjustPID(m_targetHood);
    m_boomBoom.runDrumShooterVelocityPID(m_targetVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    updateError();
    return Math.abs(error) <= tolerance;
  }
}
