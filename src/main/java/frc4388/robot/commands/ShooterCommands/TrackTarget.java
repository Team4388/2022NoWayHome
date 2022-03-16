// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.ShooterCommands;

import java.util.ArrayList;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.Vision;
import frc4388.robot.subsystems.VisionOdometry;
import frc4388.utility.desmos.DesmosServer;

public class TrackTarget extends CommandBase {
  /** Creates a new TrackTarget. */
  Turret m_turret;
  VisionOdometry m_visionOdometry;
  BoomBoom m_boomBoom;
  Hood m_hood;

  // use odometry to find x and y later
  double x;
  double y;
  double distance;
  double vel;
  double hood;
  double average;
  double output;
  Pose2d pos = new Pose2d();
  ArrayList<Point> points = new ArrayList<>();

  double m=0;
  double b=0;
  boolean isExecuted = false;

  // public static Gains m_aimGains;

  public TrackTarget (Turret turret, BoomBoom boomBoom, Hood hood, VisionOdometry visionOdometry) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_boomBoom = boomBoom;
    m_hood = hood;
    m_visionOdometry = visionOdometry;

    addRequirements(m_turret, m_boomBoom, m_hood, m_visionOdometry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = 0;
    y = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_targetAngle = angleToCenter(x, y, m_drive.getRegGyro().getDegrees());
    try {
      m_visionOdometry.setLEDs(true);
      points = m_visionOdometry.getTargetPoints();
      for(int i = 0; i < points.size(); i++) {
        DesmosServer.putPoint("Point" + i, points.get(i));
      }

      Point average = VisionOdometry.averagePoint(points);
      DesmosServer.putPoint("average", average);

      output = (average.x - VisionConstants.LIME_HIXELS/2.d) / VisionConstants.LIME_HIXELS;
      output *= 4;
      // output *= 0.5;
      DesmosServer.putDouble("output", output);
      m_turret.runTurretWithInput(output);

      double y_rot = average.y / VisionConstants.LIME_VIXELS;
      y_rot *= Math.toRadians(VisionConstants.V_FOV);
      y_rot -= Math.toRadians(VisionConstants.V_FOV) / 2;
      y_rot += Math.toRadians(VisionConstants.LIME_ANGLE);

      double distance = (VisionConstants.TARGET_HEIGHT - VisionConstants.LIME_HEIGHT) / Math.tan(y_rot);
      DesmosServer.putDouble("distance", distance);

      updateRegressionDesmos();
      double regressedDistance = distanceRegression(distance);
      regressedDistance += VisionConstants.EDGE_TO_CENTER + VisionConstants.LIMELIGHT_RADIUS;
      DesmosServer.putDouble("distanceReg", regressedDistance);

      //Vision odemetry circle fit based pose estimate
      // Point targetOffset = m_visionOdometry.getTargetOffset();
      // DesmosServer.putPoint("targetOff", targetOffset);

      vel = m_boomBoom.getVelocity(regressedDistance + 30);
      hood = m_boomBoom.getHood(regressedDistance + 30);
      // m_boomBoom.runDrumShooter(vel);
      m_boomBoom.runDrumShooterVelocityPID(vel);
      m_hood.runAngleAdjustPID(hood);

      SmartDashboard.putNumber("Regressed Distance", regressedDistance);
      SmartDashboard.putNumber("Distance", distance);
      SmartDashboard.putNumber("Hood Target Angle Track", hood);
      SmartDashboard.putNumber("Vel Target Track", vel);


      // isExecuted = true;
    }
    catch (Exception e){
      e.printStackTrace();
      // System.err.println("Exception: " + e.toString() + ", Line 78 at TrackTarget.java");
    }
    // m_turret.runshooterRotatePID(m_targetAngle);
  }

  public final double distanceRegression(double distance) {
    return (79.6078 * Math.pow(1.01343, distance) - 56.6671);
  }

  public void updateRegressionDesmos() {
    m = DesmosServer.readDouble("m");
    b = DesmosServer.readDouble("b");

    DesmosServer.putArray("MB", m, b);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return isExecuted && Math.abs(output) < .1;
  }
}
