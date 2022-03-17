// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.SwerveDrive;

/** Aarav's simple VelocityCorrection class for shooting while moving.
 * @author Aarav Shah
 */
public class VelocityCorrection {

  // subsystems
  private SwerveDrive swerve;
  private BoomBoom drum;

  // vectors (in ft and ft/sec)
  public Vector2D position;
  public Vector2D velocity;

  // scalars (in ft and sec)
  public double radius;
  public double duration;

  // target (in ft)
  public Vector2D target;

  public VelocityCorrection(SwerveDrive swerve, BoomBoom boomBoom) {

    this.swerve = swerve;
    this.drum = boomBoom;

    this.position = new Vector2D(this.swerve.getOdometry().getX(), this.swerve.getOdometry().getY());
    this.velocity = new Vector2D(this.swerve.getChassisSpeeds().vxMetersPerSecond, this.swerve.getChassisSpeeds().vyMetersPerSecond);

    this.radius = this.position.magnitude();
    this.duration = this.drum.getDuration(this.radius);

    this.target = Vector2D.multiply(this.velocity, -1 * this.duration);
  }
}
