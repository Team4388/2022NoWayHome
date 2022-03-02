// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;

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
  public double kP, kI, kD;
  public double proportional, integral, derivative;
  public double time;
  public double output;
  public double tolerance = 5.0;

  // // dummy motor
  // public WPI_TalonFX dummy = new WPI_TalonFX(69 - 420);
  // public TalonFXConfiguration dummyConfiguration = new TalonFXConfiguration();

  /** Creates a new Shoot. */
  public Shoot(SwerveDrive sDrive, BoomBoom sShooter, Turret sTurret, Hood sHood) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = sDrive;
    m_boomBoom = sShooter;
    m_turret = sTurret;
    m_hood = sHood;
    
    addRequirements(m_swerve, m_boomBoom, m_turret, m_hood);

    kP = 0.1;
    kI = 0.0;
    kD = 0.0;

    proportional = 0;
    integral = 0;
    derivative = 0;
    time = 0.02;
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
    m_odoX = 0; //TODO: get this value using odometry
    m_odoY = 0; //TODO: get this value using odometry
    m_distance = Math.sqrt(Math.pow(m_odoX, 2) + Math.pow(m_odoY, 2));

    m_gyroAngle = m_swerve.getRegGyro().getDegrees();

    // get targets (shooter tables)
    m_targetVel = m_boomBoom.getVelocity(m_distance);
    m_targetHood = m_boomBoom.getHood(m_distance);
    m_targetAngle = ((Math.atan2(m_odoY, m_odoX) * (180./Math.PI) - m_gyroAngle) + 180. + 360.) % 360.;
    m_driveTargetAngle = m_targetAngle + m_turret.getBoomBoomAngleDegrees();

    // // normal (i think) PID stuff
    // dummyConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    // dummyConfiguration.remoteFilter0.remoteSensorDeviceID = dummy.getDeviceID();
    // dummyConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;

    // dummyConfiguration.slot0.kP = 0.1;
    // dummyConfiguration.slot0.kI = 0;
    // dummyConfiguration.slot0.kD = 0;
    // dummyConfiguration.slot0.kF = 0;

    // // weird PID stuff 
    // dummyConfiguration.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SoftwareEmulatedSensor.toFeedbackDevice();
    // dummyConfiguration.remoteFilter1.remoteSensorDeviceID = ShooterConstants.TURRET_MOTOR_CAN_ID;
    // dummyConfiguration.remoteFilter1.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;
    // // dummyConfiguration.auxiliaryPID.selectedFeedbackCoefficient = 0;
  
    // dummyConfiguration.slot1.kP = 0.1;
    // dummyConfiguration.slot1.kI = 0;
    // dummyConfiguration.slot1.kD = 0;
    // dummyConfiguration.slot1.kF = 0;

    // dummy.configAllSettings(dummyConfiguration);

    // initial error
    updateError();
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
    // dummy.selectProfileSlot(0, 0);
    // dummy.selectProfileSlot(1, 1);
    // dummy.set(TalonFXControlMode.Position, m_driveTargetAngle, DemandType.AuxPID, m_targetAngle);
    // m_swerve.driveWithInput(0, 0, m_driveTargetAngle, true);
    // m_swerve.driveWithInput(0, 0, Math.cos(m_driveTargetAngle), Math.sin(m_driveTargetAngle), true); // only works for new DWI in swerve branch

    // custom pid
    runPID();
    m_swerve.driveWithInput(0, 0, output, true);
    
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
