// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class VelocityCorrection {

  SwerveDrive swerve;
  BoomBoom boomBoom;

  // vectors
  public Vector2d position;
  public Vector2d cartesianVelocity;
  public Vector2d tangentialVelocity, unitTangentialVelocity;
  public Vector2d radialVelocity, unitRadialVelocity;

  double radius;

  // find
  public Vector2d target;

  public VelocityCorrection(SwerveDrive swerve, BoomBoom boomBoom) {

    this.swerve = swerve;
    this.boomBoom = boomBoom;

    position = new Vector2d(5, 0);//new Vector2d(this.swerve.getOdometry().getX(), this.swerve.getOdometry().getY());
    cartesianVelocity = new Vector2d(1, 0);//new Vector2d(this.swerve.getChassisSpeeds().vxMetersPerSecond, this.swerve.getChassisSpeeds().vyMetersPerSecond);

    tangentialVelocity = getTangentialVelocity();
    radialVelocity = getRadialVelocity();

    radius = position.magnitude();

    target = getTargetPoint();
  }

  private Vector2d getRadialVelocity() {
    Vector2d ret = VelocityCorrection.subtract(cartesianVelocity, tangentialVelocity); 
    unitRadialVelocity = VelocityCorrection.multiply(ret, (1/ret.magnitude()));
    return ret;
  }

  private Vector2d getTangentialVelocity() {

    double hubElevation = Math.atan2(position.y, position.x);
    unitTangentialVelocity = new Vector2d(Math.cos(hubElevation - (Math.PI/2)), Math.sin(hubElevation - (Math.PI/2)));
    double tangentialVelocityMag = cartesianVelocity.scalarProject(unitTangentialVelocity);

    return VelocityCorrection.multiply(unitTangentialVelocity, tangentialVelocityMag);
  }

  private Vector2d getTargetPoint() {

    double drumVelocity = boomBoom.getVelocity(radius);
    double ballVelocity = drumVelocity; // TODO: convert from drum velocity to actual ball velocity 
    double approxShotTime = radius / ballVelocity; // TODO: better approximation to get shot time (physics/calculus?)

    Vector2d tangentialTargetPoint = VelocityCorrection.multiply(unitTangentialVelocity, -1 * approxShotTime);
    Vector2d radialTargetPoint = VelocityCorrection.multiply(unitRadialVelocity, -1 * approxShotTime);
    Vector2d targetPoint = VelocityCorrection.add(tangentialTargetPoint, radialTargetPoint);

    return targetPoint;
  }

  public static Vector2d add(Vector2d v1, Vector2d v2) {
    return new Vector2d(v1.x + v2.x, v1.y + v2.y);
  }

  public static Vector2d subtract(Vector2d v1, Vector2d v2) {
    return new Vector2d(v1.x - v2.x, v1.y - v2.y);
  }

  public static Vector2d multiply(Vector2d v1, double scalar) {
    return new Vector2d(scalar * v1.x, scalar * v1.y);
  }

  public static Vector2d round(Vector2d v) {
    v = VelocityCorrection.multiply(v, 1000);
    v.x = Math.round(v.x);
    v.y = Math.round(v.y);
    v.x = v.x / 1000;
    v.y = v.y / 1000;
    return v;
  }

}
