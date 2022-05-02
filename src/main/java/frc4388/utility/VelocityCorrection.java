// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class VelocityCorrection {

  SwerveDrive swerve;
  BoomBoom boomBoom;

  // vectors (in ft and ft/sec)
  public Vector2dExt position;
  public Vector2dExt cartesianVelocity;

  // find
  public Vector2dExt target;

  public VelocityCorrection(SwerveDrive swerve, BoomBoom boomBoom) {

    this.swerve = swerve;
    this.boomBoom = boomBoom;

    position = new Vector2dExt(5, 0);//new Vector2D(this.swerve.getOdometry().getX(), this.swerve.getOdometry().getY());
    cartesianVelocity = new Vector2dExt(-2, 3);//new Vector2D(this.swerve.getChassisSpeeds().vxMetersPerSecond, this.swerve.getChassisSpeeds().vyMetersPerSecond);

    target = getTargetPoint();
  }

  private Vector2dExt getTargetPoint() {
    double approxShotTime = 1; // TODO: get shot time from shooter tables
    
    Vector2dExt targetPoint = Vector2dExt.multiply(this.cartesianVelocity, -1 * approxShotTime);
    
    return Vector2dExt.round(targetPoint, 5);
  }

}
