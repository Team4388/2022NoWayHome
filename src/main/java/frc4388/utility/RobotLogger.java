package frc4388.utility;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class RobotLogger {
  private static final int SAMPLE_BASE = 15_000; // ms of sampling
  private static final int SAMPLE_RATE = 20; // ms between samples

  private static RobotLogger instance = null;

  public static RobotLogger getInstance() {
    return Objects.requireNonNullElseGet(instance, () -> instance = new RobotLogger());
  }

  public static double getTime() {
    return DriverStation.getMatchTime();
  }

  private RobotLogger() {
    data = new ArrayList<>();
  }

  private final List<PathPlannerTrajectory.PathPlannerState> data;
  private boolean enabled = false;

  public void setEnabled(boolean value) {
    enabled = value;
  }

  double lastVelocityMetersPerSecond = 0;
  public void put(
    double velocityMetersPerSecond,
    Pose2d poseMeters,
    double curvatureRadPerMeter,
    double positionMeters,
    Rotation2d angularVelocity,
    Rotation2d angularAcceleration,
    Rotation2d holonomicRotation) {
    if (enabled) {
      double accelerationMetersPerSecondSq = lastVelocityMetersPerSecond == 0 ? velocityMetersPerSecond : 
      (velocityMetersPerSecond - lastVelocityMetersPerSecond) / 2.0;
      PathPlannerTrajectory.PathPlannerState pathPlannerState = new PathPlannerTrajectory.PathPlannerState();
      pathPlannerState.timeSeconds = getTime();
      pathPlannerState.velocityMetersPerSecond = velocityMetersPerSecond;
      pathPlannerState.accelerationMetersPerSecondSq = accelerationMetersPerSecondSq;
      pathPlannerState.poseMeters = poseMeters;
      pathPlannerState.curvatureRadPerMeter = curvatureRadPerMeter;
      pathPlannerState.positionMeters = positionMeters;
      pathPlannerState.angularVelocity = angularVelocity;
      pathPlannerState.angularAcceleration = angularAcceleration;
      pathPlannerState.holonomicRotation = holonomicRotation;
      data.add(pathPlannerState);
      lastVelocityMetersPerSecond = velocityMetersPerSecond;
    }
  }

  public String exportPath() throws IOException {
    List<PathPlannerTrajectory.PathPlannerState> states = data;
    String pathPath = "recording." + System.currentTimeMillis() + ".json";
    Path outputPath = Filesystem.getDeployDirectory().toPath().resolve(pathPath);
    outputPath.toFile().createNewFile();
    PathPlannerTrajectoryUtil.toPathweaverJson(states, outputPath);
    return pathPath;
  }
}
