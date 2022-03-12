// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class VelocityCorrection {

  SwerveDrive swerve;
  BoomBoom boomBoom;

  // vectors (in ft and ft/sec)
  public Vector2D position;
  public Vector2D cartesianVelocity;

  // find
  public Vector2D target;

  public VelocityCorrection(SwerveDrive swerve, BoomBoom boomBoom) {

    this.swerve = swerve;
    this.boomBoom = boomBoom;

    position = new Vector2D(5, 0);//new Vector2D(this.swerve.getOdometry().getX(), this.swerve.getOdometry().getY());
    cartesianVelocity = new Vector2D(-2, 3);//new Vector2D(this.swerve.getChassisSpeeds().vxMetersPerSecond, this.swerve.getChassisSpeeds().vyMetersPerSecond);

    target = getTargetPoint();
  }

  private Vector2D getTargetPoint() {
    double approxShotTime = 1; // TODO: get shot time from shooter tables
    
    Vector2D targetPoint = Vector2D.multiply(this.cartesianVelocity, -1 * approxShotTime);
    
    return Vector2D.round(targetPoint, 5);
  }

}
