package frc4388.robot.commands;

import java.io.File;
import java.io.IOException;
import java.io.StringWriter;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.StandardWatchEventKinds;
import java.nio.file.WatchEvent;
import java.nio.file.WatchKey;
import java.time.Clock;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import com.diffplug.common.base.Errors;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.PathPlannerUtil;
import frc4388.utility.PathPlannerUtil.Path.Waypoint;
import frc4388.utility.shuffleboard.ListeningSendableChooser;

public class PathRecorder extends CommandBase {
  private static final Logger LOGGER = Logger.getLogger(PathRecorder.class.getSimpleName());
  private static double PATH_POLLING_PERIOD = 0.5;
  private static final Clock SYSTEM_CLOCK = Clock.system(ZoneId.systemDefault());
  private static final Path PATHPLANNER_DIRECTORY = Filesystem.getDeployDirectory().toPath().resolve("pathplanner");
  private static final String PATHPLANNER_EXTENSION = ".path";
  private static final DateTimeFormatter RECORDING_FILE_NAME_FORMATTER = DateTimeFormatter.ofPattern("'Recording' yyyy-MM-dd HH-mm-ss.SSS'.path'");
  // Function that removes the ".path" from the end of a string.
  private static final Function<CharSequence, String> PATH_EXTENSION_REMOVER = ((Function<CharSequence, Matcher>) Pattern.compile(PATHPLANNER_EXTENSION)::matcher).andThen(m -> m.replaceFirst(""));

  private PathPlannerTrajectory loadedPathTrajectory = null;
  private WatchKey m_watchKey;
  private final ListeningSendableChooser<File> pathChooser = new ListeningSendableChooser<>(this::loadPath);
  private final List<Waypoint> pathPoints = new ArrayList<>();
  private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private final NetworkTable recordingNetworkTable = networkTableInstance.getTable("Recording");
  private final SwerveDrive m_swerveDrive;
  private final CommandBase m_pathWatcher = new NotifierCommand(this::updatePathChooser, PATH_POLLING_PERIOD) {
    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }.withName("Path Watcher");
  private final CommandBase m_pathSaver = new InstantCommand(this::savePath) {
    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }.withName("Save Recording");

  public PathRecorder(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
    setName("Record Path (Cancel to Save)");
  }

  /**
   * Creates a WatchKey for the path planner directory and registers it with the WatchService. Then
   * creates a NotifierCommand that will update the path chooser with the latest path files. Finally,
   * adds the existing path files to the path chooser
   */
  @Override
  public void initialize() {
    try {
      m_watchKey = PATHPLANNER_DIRECTORY.register(FileSystems.getDefault().newWatchService(), StandardWatchEventKinds.ENTRY_CREATE, StandardWatchEventKinds.ENTRY_MODIFY, StandardWatchEventKinds.ENTRY_DELETE);
      m_pathWatcher.schedule();
    } catch (IOException exception) {
      LOGGER.log(Level.SEVERE, "Exception with path file watcher.", exception);
    }
    Arrays.stream(PATHPLANNER_DIRECTORY.toFile().listFiles()).filter(file -> file.getName().endsWith(PATHPLANNER_EXTENSION)).sorted(Comparator.comparingLong(File::lastModified)).forEachOrdered(file -> pathChooser.addOption(file.getName(), file));
    SmartDashboard.putData("Path Chooser", pathChooser);
  }

  /**
   * Period loop to run during recording. Gets the estimated position and rotation of the robot and
   * saves it as a new waypoint.
   */
  @Override
  public void execute() {
    Translation2d position = m_swerveDrive.m_odometry.getPoseMeters().getTranslation();
    Rotation2d rotation = m_swerveDrive.m_gyro.getRotation2d();
    // FIXME: Chassis speeds are created from joystick inputs and do not reflect actual robot velocity.
    Translation2d velocity = new Translation2d(m_swerveDrive.getChassisSpeeds().vxMetersPerSecond, m_swerveDrive.getChassisSpeeds().vyMetersPerSecond);
    Waypoint waypoint = new Waypoint(position, position, position, rotation.getDegrees(), false, SwerveDriveConstants.PATH_RECORD_VELOCITY ? velocity.getNorm() : null, false);
    pathPoints.add(waypoint);
  }

  @Override
  public void schedule() {
    super.schedule();
    LOGGER.info("Begin recording.");
  }

  @Override
  public void end(boolean interrupted) {
    LOGGER.info("End recording.");
    m_pathSaver.schedule();
  }

  public PathPlannerTrajectory getPath() {
    return loadedPathTrajectory;
  }

  /**
   * Called when a file is created, modified, or deleted. Adds newly created .path files to the
   * SendableChooser. Reloads the path if the currently selected file is modified.
   */
  private void updatePathChooser() {
    if (m_watchKey != null) {
      List<WatchEvent<?>> watchEvents = m_watchKey.pollEvents();
      if (!watchEvents.isEmpty()) {
        List<WatchEvent<?>> pathWatchEvents = watchEvents.stream().filter(e -> e.kind().type().isAssignableFrom(Path.class)).collect(Collectors.toList());
        for (WatchEvent<?> pathWatchEvent : pathWatchEvents) {
          Path watchEventPath = (Path) pathWatchEvent.context();
          File watchEventFile = watchEventPath.toFile();
          String watchEventFileName = watchEventFile.getName();
          if (watchEventFileName.endsWith(PATHPLANNER_EXTENSION)) {
            if (pathWatchEvent.kind().equals(StandardWatchEventKinds.ENTRY_CREATE)) {
              LOGGER.log(Level.WARNING, "PathPlanner file {0} created. Options added to SendableChooser.", watchEventFileName);
              pathChooser.addOption(watchEventFile.getName(), watchEventFile);
            } else if (pathWatchEvent.kind().equals(StandardWatchEventKinds.ENTRY_MODIFY)) {
              LOGGER.log(Level.WARNING, "PathPlanner file {0} modified.", watchEventFileName);
              if (watchEventFileName.equals(pathChooser.getSelected().getName())) {
                LOGGER.log(Level.SEVERE, "PathPlanner file {0} already selected. Reloading path.", watchEventFileName);
                loadPath(watchEventFileName);
              }
            } else if (pathWatchEvent.kind().equals(StandardWatchEventKinds.ENTRY_DELETE)) {
              LOGGER.log(Level.SEVERE, "PathPlanner file {0} deleted. Removing options from SendableChooser not yet implemented.", watchEventFileName);
            }
          }
        }
      }
      if (!m_watchKey.reset()) LOGGER.severe("File watch key invalid.");
    }
  }

  private void loadPath(String pathName) {
    LOGGER.warning("Unloading path.");
    loadedPathTrajectory = null;
    LOGGER.info("Done unloading.");
    if (pathName != null) {
      LOGGER.log(Level.WARNING, "Loading path {0}.", pathName);
      loadedPathTrajectory = PathPlanner.loadPath(PATH_EXTENSION_REMOVER.apply(pathName), SwerveDriveConstants.PATH_MAX_VELOCITY, SwerveDriveConstants.PATH_MAX_ACCELERATION);
      m_swerveDrive.m_field.getObject("traj").setTrajectory(loadedPathTrajectory);
      LOGGER.info("Done loading.");
    } else {
      LOGGER.severe("No path to load.");
    }
  }

  private void savePath() {
    // INFO: Had to chown the pathplanner folder in order to save paths.
    File outputFile = PATHPLANNER_DIRECTORY.resolve(ZonedDateTime.now(SYSTEM_CLOCK).format(RECORDING_FILE_NAME_FORMATTER)).toFile();
    LOGGER.log(Level.WARNING, "Creating path {0}.", outputFile.getPath());
    if (!pathPoints.isEmpty() && Boolean.TRUE.equals(Errors.log().getWithDefault(outputFile::createNewFile, false))) {

      // Remove points whose angles to neighboring points are less than 10 degrees apart.
      int j = 0;
      for (int i = 1; i < pathPoints.size() - 1; i++) {
        var prev = pathPoints.get(j).anchorPoint.orElseThrow();
        var current = pathPoints.get(i).anchorPoint.orElseThrow();
        var next = pathPoints.get(i + 1).anchorPoint.orElseThrow();
        var fromPrevious = current.minus(prev);
        var toNext = next.minus(current);
        var angleFromPrevious = new Rotation2d(fromPrevious.getX(), fromPrevious.getY());
        var angleToNext = new Rotation2d(toNext.getX(), toNext.getY());
        if (Math.abs(angleFromPrevious.minus(angleToNext).getDegrees()) < SwerveDriveConstants.MIN_WAYPOINT_ANGLE || (next.getDistance(prev) < SwerveDriveConstants.MIN_WAYPOINT_DISTANCE && pathPoints.get(i).velOverride.map(v -> v < SwerveDriveConstants.MIN_WAYPOINT_VELOCITY).orElse(false))) pathPoints.set(i, null);
        else j = i;
      }
      pathPoints.removeIf(Objects::isNull);
      // Make control points
      pathPoints.get(0).nextControl = Optional.of(makeControlPoints(null, pathPoints.get(0).anchorPoint.orElseThrow(), pathPoints.get(1).anchorPoint.orElseThrow()).getSecond());
      for (int i = 1; i < pathPoints.size() - 1; i++) {
        var controls = makeControlPoints(pathPoints.get(i - 1).anchorPoint.orElseThrow(), pathPoints.get(i).anchorPoint.orElseThrow(), pathPoints.get(i + 1).anchorPoint.orElseThrow());
        pathPoints.get(i).prevControl = Optional.of(controls.getFirst());
        pathPoints.get(i).nextControl = Optional.of(controls.getSecond());
      }
      pathPoints.get(pathPoints.size() - 1).prevControl = Optional.of(makeControlPoints(pathPoints.get(pathPoints.size() - 2).anchorPoint.orElseThrow(), pathPoints.get(pathPoints.size() - 1).anchorPoint.orElseThrow(), null).getFirst());
      // Create the path
      PathPlannerUtil.Path path = new PathPlannerUtil.Path();
      path.waypoints = Optional.ofNullable(pathPoints.toArray(PathPlannerUtil.Path.Waypoint[]::new));
      // TODO: Change to use measured maximum velocity and acceleration.
      path.maxVelocity = Optional.ofNullable(null);
      path.maxAcceleration = Optional.ofNullable(null);
      path.isReversed = Optional.ofNullable(false);
      pathPoints.clear();

      if (RobotBase.isReal()) path.write(outputFile);
      StringWriter writer = new StringWriter();
      path.write(writer);
      recordingNetworkTable.getEntry(outputFile.getName()).setString(writer.toString());
      pathChooser.setDefaultOption(outputFile.getName(), outputFile);
      LOGGER.log(Level.INFO, "Recorded path to {0}.", outputFile.getPath());
    } else LOGGER.log(Level.SEVERE, "Unable to record path to {0}", outputFile.getPath());
  }

  private static Pair<Translation2d, Translation2d> makeControlPoints(Translation2d prev, Translation2d current, Translation2d next) {
    var line = Objects.requireNonNullElse(next, current).minus(Objects.requireNonNullElse(prev, current)).div(4);
    return Pair.of(current.minus(line), current.plus(line));
  }
}
