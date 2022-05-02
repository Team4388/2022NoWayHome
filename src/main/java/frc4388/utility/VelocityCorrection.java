package frc4388.utility;

import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class VelocityCorrection {

  SwerveDrive swerve;
  BoomBoom boomBoom;

  // vectors (in ft and ft/sec)
  public FRC4388_Vector2d position;
  public FRC4388_Vector2d cartesianVelocity;

  // find
  public FRC4388_Vector2d target;

  public VelocityCorrection(SwerveDrive swerve, BoomBoom boomBoom) {

    this.swerve = swerve;
    this.boomBoom = boomBoom;

    position = new FRC4388_Vector2d(5, 0);//new Vector2D(this.swerve.getOdometry().getX(), this.swerve.getOdometry().getY());
    cartesianVelocity = new FRC4388_Vector2d(-2, 3);//new Vector2D(this.swerve.getChassisSpeeds().vxMetersPerSecond, this.swerve.getChassisSpeeds().vyMetersPerSecond);

    target = getTargetPoint();
  }

  private FRC4388_Vector2d getTargetPoint() {
    double approxShotTime = 1; // TODO: get shot time from shooter tables
    
    FRC4388_Vector2d targetPoint = FRC4388_Vector2d.multiply(this.cartesianVelocity, -1 * approxShotTime);
    
    return FRC4388_Vector2d.round(targetPoint, 5);
  }

}
