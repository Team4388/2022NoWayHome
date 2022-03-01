// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.StandardWatchEventKinds;
import java.nio.file.WatchEvent;
import java.nio.file.WatchKey;
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
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.LEDPatterns;
import frc4388.utility.ListeningSendableChooser;
import frc4388.utility.PathPlannerUtil;
import frc4388.utility.PathPlannerUtil.Path.Waypoint;
import frc4388.utility.controller.DeadbandedXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final Logger LOGGER = Logger.getLogger(RobotContainer.class.getSimpleName());
  /* RobotMap */
  private final RobotMap m_robotMap = new RobotMap();

  /* Subsystems */
  private final SwerveDrive m_robotSwerveDrive = new SwerveDrive(
      m_robotMap.leftFront, m_robotMap.leftBack, m_robotMap.rightFront, m_robotMap.rightBack, m_robotMap.gyro);

  private final LED m_robotLED = new LED(m_robotMap.LEDController);

  /* Controllers */
  private final XboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
  private final XboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);

  private PathPlannerTrajectory loadedPathTrajectory = null;
  private static final Function<CharSequence, String> pathExtensionRemover = ((Function<CharSequence, Matcher>) Pattern
      .compile(".path")::matcher).andThen(m -> m.replaceFirst(""));

  private final ListeningSendableChooser<File> autoChooser = new ListeningSendableChooser<>(this::loadPath);

  private void loadPath(String pathName) {
    LOGGER.warning("Loading path " + pathName);
    loadedPathTrajectory = null;
    loadedPathTrajectory = PathPlanner.loadPath(pathExtensionRemover.apply(Objects.requireNonNullElse(pathName, "")),
        5.5, 50);
    LOGGER.info("Done loading");
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    /* Default Commands */
    // drives the swerve drive with a two-axis input from the driver controller
    m_robotSwerveDrive.setDefaultCommand(
        new RunCommand(() -> m_robotSwerveDrive.driveWithInput(
            getDriverController().getLeftX(),
            getDriverController().getLeftY(),
            getDriverController().getRightX(),
            getDriverController().getRightY(),
            true),
            m_robotSwerveDrive).withName("Swerve driveWithInput defaultCommand"));

    // continually sends updates to the Blinkin LED controller to keep the lights on
    m_robotLED.setDefaultCommand(new RunCommand(m_robotLED::updateLED, m_robotLED).withName("LED update defaultCommand"));

    try {
      WatchKey watchKey = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").register(FileSystems.getDefault().newWatchService(), StandardWatchEventKinds.ENTRY_CREATE, StandardWatchEventKinds.ENTRY_MODIFY, StandardWatchEventKinds.ENTRY_DELETE);
      // Save this and other commands as fields so they can be rescheduled.
      new NotifierCommand(() -> {
        List<WatchEvent<?>> watchEvents = watchKey.pollEvents();
        if (!watchEvents.isEmpty()) {
          List<WatchEvent<?>> pathWatchEvents = watchEvents.stream().filter(e -> e.kind().type().isAssignableFrom(Path.class)).collect(Collectors.toList());
          for (WatchEvent<?> pathWatchEvent : pathWatchEvents) {
            Path watchEventPath = (Path) pathWatchEvent.context();
            File watchEventFile = watchEventPath.toFile();
            String watchEventFileName = watchEventFile.getName();
            if (watchEventFileName.endsWith(".path")) {
              if (pathWatchEvent.kind().equals(StandardWatchEventKinds.ENTRY_CREATE)) {
                LOGGER.warning("PathPlanner file " + watchEventFileName + " created. Options added to SendableChooser.");
                autoChooser.addOption(watchEventFile.getName(), watchEventFile);
              } else if (pathWatchEvent.kind().equals(StandardWatchEventKinds.ENTRY_MODIFY)) {
                LOGGER.warning("PathPlanner file " + watchEventFileName + " modified.");
                if (watchEventFileName.equals(autoChooser.getSelected().getName())) {
                  LOGGER.severe("PathPlanner file " + watchEventFileName + " already selected. Reloading path.");
                  loadPath(watchEventFileName);
                }
              } else if (pathWatchEvent.kind().equals(StandardWatchEventKinds.ENTRY_DELETE)) {
                LOGGER.severe("PathPlanner file " + watchEventFileName + " deleted. Removing options from SendableChooser not yet implemented.");
              }
            }
          }
        }
        if (!watchKey.reset())
          LOGGER.severe("File watch key invalid.");
      }, 0.5) {
        @Override
        public boolean runsWhenDisabled() {
          return true;
        }
      }.withName("Path Watcher").schedule();
    } catch (IOException exception) {
      LOGGER.log(Level.SEVERE, "Exception with path file watcher.", exception);
    }
    Arrays.stream(Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toFile().listFiles())
        .filter(file -> file.getName().endsWith(".path")).sorted(Comparator.comparingLong(File::lastModified))
        .forEachOrdered(file -> autoChooser.addOption(file.getName(), file));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    recordInit();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    new JoystickButton(getDriverController(), XboxController.Button.kY.value)
        // new XboxControllerRawButton(m_driverXbox, XboxControllerRaw.Y_BUTTON)
        .whenPressed(m_robotSwerveDrive.m_gyro::reset);

    new JoystickButton(getDriverController(), XboxController.Button.kLeftBumper.value)
        // new XboxControllerRawButton(m_driverXbox,
        // XboxControllerRaw.LEFT_BUMPER_BUTTON)
        .whenPressed(() -> m_robotSwerveDrive.highSpeed(false));

    new JoystickButton(getDriverController(), XboxController.Button.kRightBumper.value)
        // new XboxControllerRawButton(m_driverXbox,
        // XboxControllerRaw.RIGHT_BUMPER_BUTTON)
        .whenPressed(() -> m_robotSwerveDrive.highSpeed(true));

    new JoystickButton(getDriverController(), XboxController.Button.kA.value)
        .whenPressed(() -> resetOdometry(new Pose2d(0, 0, new Rotation2d(0))));

    /* Operator Buttons */
    // activates "Lit Mode"
    new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
        // new XboxControllerRawButton(m_driverXbox, XboxControllerRaw.A_BUTTON)
        .whenPressed(() -> m_robotLED.setPattern(LEDPatterns.LAVA_RAINBOW))
        .whenReleased(() -> m_robotLED.setPattern(LEDConstants.DEFAULT_PATTERN));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (loadedPathTrajectory != null) {
      PIDController xController = SwerveDriveConstants.X_CONTROLLER;
      PIDController yController = SwerveDriveConstants.Y_CONTROLLER;
      ProfiledPIDController thetaController = SwerveDriveConstants.THETA_CONTROLLER;
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      PathPlannerState initialState = loadedPathTrajectory.getInitialState();
      Pose2d initialPosition = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
      return new SequentialCommandGroup(
          new InstantCommand(m_robotSwerveDrive.m_gyro::reset),
          new InstantCommand(() -> m_robotSwerveDrive.resetOdometry(initialPosition)),
          new PPSwerveControllerCommand(loadedPathTrajectory, m_robotSwerveDrive::getOdometry,
              m_robotSwerveDrive.m_kinematics, xController, yController, thetaController,
              m_robotSwerveDrive::setModuleStates, m_robotSwerveDrive),
          new InstantCommand(m_robotSwerveDrive::stopModules)).withName("Run Autonomous Path");
    } else {
      LOGGER.severe("No auto selected.");
      return new RunCommand(() -> {}).withName("No Autonomous Path");
    }
  }

  public XboxController getDriverController() {
    return m_driverXbox;
  }

  public Pose2d getOdometry() {
    return m_robotSwerveDrive.getOdometry();
  }

  public void resetOdometry(Pose2d pose) {
    m_robotSwerveDrive.resetOdometry(pose);
  }

  public XboxController getOperatorController() {
    return m_operatorXbox;
  }

  private final List<Waypoint> pathPoints = new ArrayList<>();

  public void recordInit() {
    SmartDashboard.putData("Recording",
        new RunCommand(this::recordPeriodic) {
          @Override
          public void end(boolean interupted) {
            new InstantCommand(RobotContainer.this::saveRecording) {
              @Override
              public boolean runsWhenDisabled() {
                return true;
              }
            }.withName("Save Recording").schedule();
          }
        }.withName("Record Path (Cancel to Save)"));
  }

  private void saveRecording() {
    // IMPORTANT: Had to chown the pathplanner folder in order to save autos.
    File outputFile = Filesystem.getDeployDirectory().toPath().resolve("pathplanner")
        .resolve("recording." + System.currentTimeMillis() + ".path").toFile();
    LOGGER.log(Level.WARNING, "Creating path {0}.", outputFile.getPath());
    if (Boolean.TRUE.equals(Errors.log().getWithDefault(outputFile::createNewFile, false))) {
      createPath(null, null, false).write(outputFile);
      autoChooser.setDefaultOption(outputFile.getName(), outputFile);
      LOGGER.log(Level.INFO, "Recorded path to {0}.", outputFile.getPath());
    } else
      LOGGER.log(Level.SEVERE, "Unable to record path to {0}", outputFile.getPath());
  }

  public void recordPeriodic() {
    Translation2d position = m_robotSwerveDrive.m_poseEstimator.getEstimatedPosition().getTranslation();
    Rotation2d rotation = m_robotSwerveDrive.m_gyro.getRotation2d();
    Translation2d velocity = new Translation2d(m_robotSwerveDrive.chassisSpeeds.vxMetersPerSecond,
        m_robotSwerveDrive.chassisSpeeds.vyMetersPerSecond);
    Waypoint waypoint = new Waypoint(position, position, position, rotation.getDegrees(), false, velocity.getNorm(),
        false);
    pathPoints.add(waypoint);
  }

  public PathPlannerUtil.Path createPath(Double maxVelocity, Double maxAcceleration, Boolean isReversed) {
    // pathPoints = Arrays.stream(PathPlannerUtil.Path.read(autoChooser.getSelected()).waypoints.get()).collect(Collectors.toList());
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
      if (Math.abs(angleFromPrevious.minus(angleToNext).getDegrees()) < 20)
        pathPoints.set(i, null);
      else
        j = i;
    }
    pathPoints.removeIf(Objects::isNull);
    // Make control points
    pathPoints.get(0).nextControl = Optional.of(makeControlPoints(null, pathPoints.get(0).anchorPoint.orElseThrow(),
        pathPoints.get(1).anchorPoint.orElseThrow()).getSecond());
    for (int i = 1; i < pathPoints.size() - 1; i++) {
      var controls = makeControlPoints(pathPoints.get(i - 1).anchorPoint.orElseThrow(),
          pathPoints.get(i).anchorPoint.orElseThrow(), pathPoints.get(i + 1).anchorPoint.orElseThrow());
      pathPoints.get(i).prevControl = Optional.of(controls.getFirst());
      pathPoints.get(i).nextControl = Optional.of(controls.getSecond());
    }
    pathPoints.get(pathPoints.size() - 1).prevControl = Optional
        .of(makeControlPoints(pathPoints.get(pathPoints.size() - 2).anchorPoint.orElseThrow(),
            pathPoints.get(pathPoints.size() - 1).anchorPoint.orElseThrow(), null).getFirst());
    // Create the path
    PathPlannerUtil.Path path = new PathPlannerUtil.Path();
    path.waypoints = Optional.ofNullable(pathPoints.toArray(PathPlannerUtil.Path.Waypoint[]::new));
    path.maxVelocity = Optional.ofNullable(maxVelocity);
    path.maxAcceleration = Optional.ofNullable(maxAcceleration);
    path.isReversed = Optional.ofNullable(isReversed);
    pathPoints.clear();
    return path;
  }

  private static Pair<Translation2d, Translation2d> makeControlPoints(Translation2d prev, Translation2d current,
      Translation2d next) {
    var line = Objects.requireNonNullElse(next, current).minus(Objects.requireNonNullElse(prev, current)).div(4);
    return Pair.of(current.minus(line), current.plus(line));
  }
}
