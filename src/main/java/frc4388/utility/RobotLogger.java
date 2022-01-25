package frc4388.utility;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public final class RobotLogger {
  private static final int SAMPLE_BASE = 15_000; // ms of sampling
  private static final int SAMPLE_RATE = 20; // ms between samples

  private static RobotLogger instance = null;

  public static RobotLogger getInstance() {
    return Objects.requireNonNullElseGet(instance, () -> instance = new RobotLogger());
  }

  private static long getTime() {
    return RobotTime.getInstance().m_robotTime;
  }

  private RobotLogger() {
    data = new HashMap<>();
  }

  private final Map<String, Map<Long, Object>> data;
  private boolean enabled = false;

  public void setEnabled(boolean value) {
    enabled = value;
  }

  public void put(String key, Object value) {
    if (enabled)
      data.compute(key, (k, v) -> {
        v = v == null ? new Hashtable<>(SAMPLE_BASE / SAMPLE_RATE, 1) : v;
        v.put(getTime(), value);
        return v;
      });
  }

  public void exportPath() throws IOException {
    List<Trajectory.State> states = data.get("poseMeters").entrySet().stream().map(entry -> {
      double timeSeconds = entry.getKey();
      double velocityMetersPerSecond = 0;
      double accelerationMetersPerSecondSq = 0;
      Pose2d poseMeters = (Pose2d) entry.getValue();
      double curvatureRadPerMeter = 0;
      return new Trajectory.State(timeSeconds, velocityMetersPerSecond, accelerationMetersPerSecondSq, poseMeters, curvatureRadPerMeter);
    }).collect(Collectors.toUnmodifiableList());
    String pathPath = "paths/" + System.nanoTime() + ".wpilib.json";
    Path outputPath = Filesystem.getDeployDirectory().toPath().resolve(pathPath);
    outputPath.toFile().createNewFile();
    TrajectoryUtil.toPathweaverJson(new Trajectory(states), outputPath);
  }
}
