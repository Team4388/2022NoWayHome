package frc4388.utility;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.utility.PathPlannerUtil.Path.Waypoint;

public final class RobotLogger {
  private static RobotLogger instance = null;

  public static RobotLogger getInstance() {
    return Objects.requireNonNullElseGet(instance, () -> instance = new RobotLogger());
  }

  public static double getTime() {
    return DriverStation.getMatchTime();
  }

  private RobotLogger() {
    sendableChooser.setDefaultOption("Disable", false);
    sendableChooser.addOption("Enable", true);
    SmartDashboard.putData("Recording", sendableChooser);
  }

  private final List<Waypoint> data = new ArrayList<>();
  private final SendableChooser<Boolean> sendableChooser = new SendableChooser<>();

  double lastVelocityMetersPerSecond = 0;
  public void put(Double anchorPointX, Double anchorPointY, Double prevControlX, Double prevControlY, Double nextControlX, Double nextControlY, Double holonomicAngle, Boolean isReversal, Double velOverride, Boolean isLocked) {
    if (Boolean.TRUE.equals(sendableChooser.getSelected())) {
      Waypoint waypoint = new Waypoint(anchorPointX, anchorPointY, prevControlX, prevControlY, nextControlX, nextControlY, holonomicAngle, isReversal, velOverride, isLocked);
      data.add(waypoint);
    }
  }

  @SuppressWarnings("unchecked")
  public PathPlannerUtil.Path createPath(Double maxVelocity, Double maxAcceleration, Boolean isReversed) {
    PathPlannerUtil.Path path = new PathPlannerUtil.Path();
    path.waypoints = data.stream().map(Optional::ofNullable).toArray(Optional[]::new);
    path.maxVelocity = Optional.ofNullable(maxVelocity);
    path.maxAcceleration = Optional.ofNullable(maxAcceleration);
    path.isReversed = Optional.ofNullable(isReversed);
    data.clear();
    return path;
  }
}
