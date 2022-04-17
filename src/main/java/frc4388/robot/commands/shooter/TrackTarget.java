package frc4388.robot.commands.shooter;

import java.util.ArrayList;
import java.util.logging.Logger;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.VisionOdometry;

// TODO: Try putting swerve drive rotation to be perpendicular to the center point to prevent being rolled while shooting.
// FIXME: Regression and distance offset happen separately, and the turret offset uses the regressed distance without the distance offset.
public class TrackTarget extends CommandBase {
  private static final Logger LOGGER = Logger.getLogger(TrackTarget.class.getSimpleName());
  private static final double TURRET_OFFSET_DEGREES = 10;

  private final VisionOdometry m_visionOdometry;
  private final Turret m_turret;
  private final Hood m_hood;
  private final BoomBoom m_boomBoom;

  public TrackTarget(VisionOdometry visionOdometry, Turret turret, Hood hood, BoomBoom boomBoom) {
    this.m_visionOdometry = visionOdometry;
    this.m_turret = turret;
    this.m_hood = hood;
    this.m_boomBoom = boomBoom;
  }

  @Override
  public void initialize() {
    m_visionOdometry.setDriverMode(false);
    m_visionOdometry.setLEDs(true);
    SmartDashboard.putBoolean("TrackTarget/Running", true);
  }

  @Override
  public void execute() {
    ArrayList<Point> points = m_visionOdometry.getTargetPoints();
    if (!points.isEmpty()) {
      Point pointAverage = VisionOdometry.averagePoint(points);

      double distanceTarget = getDistance(pointAverage.y);
      double turretOffset = Math.toDegrees(Math.atan(TURRET_OFFSET_DEGREES / distanceTarget)) * VisionConstants.PIXELS_PER_DEGREE;
      distanceTarget -= SmartDashboard.getNumber("TrackTarget/Target Distance Offset", -35);
      SmartDashboard.putNumber("TrackTarget/Target Distance (Regressed and Offset)", distanceTarget);

      double turretOutput = 2.1 * ((pointAverage.x + 40) - VisionConstants.LIME_HIXELS / 2.0) / VisionConstants.LIME_HIXELS;
      double hoodTarget = m_boomBoom.getHood(distanceTarget);
      double velocityTarget = m_boomBoom.getVelocity(distanceTarget);
      SmartDashboard.putNumber("TrackTarget/Target Hood", hoodTarget);
      SmartDashboard.putNumber("TrackTarget/Target Velocity", velocityTarget);

      m_turret.runTurretWithInput(turretOutput - turretOffset);
      m_hood.runAngleAdjustPID(hoodTarget);
      m_boomBoom.runDrumShooterVelocityPID(velocityTarget);
    } else {
      LOGGER.severe("No vision target points.");
      SmartDashboard.putString("TrackTarget/Target Distance (Regressed and Offset)", "obscured");
      SmartDashboard.putString("TrackTarget/Target Hood", "obscured");
      SmartDashboard.putString("TrackTarget/Target Velocity", "obscured");
    }
  }
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("TrackTarget/Running", false);
  }
  private static double getDistance(double averageY) {
    double yRot = averageY / VisionConstants.LIME_VIXELS;
    yRot *= Math.toRadians(VisionConstants.V_FOV);
    yRot -= Math.toRadians(VisionConstants.V_FOV) / 2;
    yRot += Math.toRadians(VisionConstants.LIME_ANGLE);
    double distance = (VisionConstants.TARGET_HEIGHT - VisionConstants.LIME_HEIGHT) / Math.tan(yRot);

    double regressedDistance = regression(distance, 79.6078, 1.01343, -56.6671);
    regressedDistance += VisionConstants.EDGE_TO_CENTER + VisionConstants.LIMELIGHT_RADIUS;
    return regressedDistance;
  }
  private static double regression(double x, double a, double b, double c) {
    return a * Math.pow(b, x) + c;
  }
}
