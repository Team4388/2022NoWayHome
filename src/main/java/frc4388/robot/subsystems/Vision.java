// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.Hood;
import frc4388.utility.controller.IHandController;

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

public Vision(Turret aimSubsystem, BoomBoom boomBoom) {
  m_turret = aimSubsystem;
  m_boomBoom = boomBoom;
  m_hood = m_boomBoom.m_hoodSubsystem;
  //addRequirements(m_turret);
  limeOff();
  changePipeline(0);
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(3);
}

public void track(){
  target = getV();
  xAngle = getX();
  yAngle = getY();

  //find distance
  distance = (VisionConstants.TARGET_HEIGHT) / Math.tan((VisionConstants.LIME_ANGLE + yAngle) * (Math.PI / 180));
  realDistance = (1.09 * distance) - 12.8;

  if (target == 1.0) { //checks if target is in view
    //aims left and right
    turnAmount = ((xAngle / VisionConstants.FOV) * VisionConstants.TURN_P_VALUE);
    if (Math.abs(xAngle) < VisionConstants.X_ANGLE_ERROR) {
      turnAmount = 0;
    } 
    else if (turnAmount > 0 && turnAmount < 0.1){
      turnAmount = 0.1;
    }
    else if (turnAmount < 0 && turnAmount > -0.1){
        turnAmount = -0.1;
    }
  }
  m_turret.turnWithJoystick(-turnAmount);

  SmartDashboard.putNumber("Disance to Target", realDistance);


  //start CSV

  fireVel = m_boomBoom.m_shooterTable.getVelocity(realDistance);
  fireAngle = m_boomBoom.m_shooterTable.getHood(realDistance);
  //fire angle unknown so far
  //end of CSV

  m_boomBoom.m_fireVel = fireVel;
  m_hood.m_fireAngle = fireAngle;
  m_turret.m_targetDistance = distance;

  checkFinished();
}

public void checkFinished(){
  if (xAngle < 0.5 && xAngle > -0.5 && target == 1){
    m_turret.m_isAimReady = true;
  }
  else{
    m_turret.m_isAimReady = false;
  }
}

public void limeOff(){
  NetworkTableInstance.getDefault.getTable("limelight").getEntry("camMode").setNumber(0);
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
}

  public void limeOn(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
}

  public void changePipeline(int pipelineId)
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineId);
  }
  
  public double getV()
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }

  public double getX()
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getY()
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }
  @Override
  public void periodic(){
    //called once per scheduler run
  }
}


