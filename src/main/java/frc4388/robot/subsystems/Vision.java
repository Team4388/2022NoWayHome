// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class Vision extends SubsystemBase {
//setup
  Turret m_turret;
BoomBoom m_boomBoom;
Hood m_hood;

NetworkTableEntry xEntry;
IHandController m_driverController;
//Aiming
double turnAmount = 0;
double xAngle = 0;
double yAngle = 0;
double target = 0;
public double distance;
public double realDistance;
public static double fireVel;
public static double fireAngle;

public double m_hoodTrim;
public double m_turretTrim;

LimeLight m_limeLight;

public TrackTarget(ShooterAim aimSubsystem, LimeLight limeLight) {
  m_turret = turretSubsystem;
  m_boomBoom = m_turret.m_turretSubsystem;
  m_hood = m_boomBoom.m_hoodSubsystem;
  m_limeLight = limeLight;
  addRequirements(m_turret);

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
