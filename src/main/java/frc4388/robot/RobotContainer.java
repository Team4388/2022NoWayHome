// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardWatchEventKinds;
import java.nio.file.WatchEvent;
import java.nio.file.WatchKey;
import java.nio.file.WatchService;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import com.diffplug.common.base.Errors;
import com.diffplug.common.base.Errors.Plugins.Log;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.LEDPatterns;
import frc4388.utility.PathPlannerUtil;
import frc4388.utility.SendableRunChooser;
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

  private String loadedPathName = "";
  private PathPlannerTrajectory loadedPathTrajectory = null;
  private static final Function<CharSequence, String> pathExtensionRemover = ((Function<CharSequence, Matcher>) Pattern
      .compile(".path")::matcher).andThen(m -> m.replaceFirst(""));

  private final SendableRunChooser<File> autoChooser = new SendableRunChooser<>(selectedAuto -> {
    if (selectedAuto != null && !selectedAuto.equals(loadedPathName)) {
      loadedPathTrajectory = PathPlanner.loadPath(pathExtensionRemover.apply(selectedAuto), 5.5, 50);
      loadedPathName = selectedAuto;
    }
  });

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  @SuppressWarnings("unchecked")
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
    try (WatchService watcher = FileSystems.getDefault().newWatchService()) {
      WatchKey watchKey = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").register(
          FileSystems.getDefault().newWatchService(),
          StandardWatchEventKinds.ENTRY_CREATE, StandardWatchEventKinds.ENTRY_MODIFY,
          StandardWatchEventKinds.ENTRY_DELETE);
      new NotifierCommand(() -> {}, TimedRobot.kDefaultPeriod) {
        @Override
        public void execute() {
          var selectedAuto = autoChooser.getSelected();
          if (selectedAuto != null && !selectedAuto.getName().equals(loadedPathName)) {
            setName("Path Watcher: Loading Path");
            // loadedPathTrajectory = PathPlanner.loadPath(pathExtensionRemover.apply(selectedAuto.getName()), 5.5, 50);
            loadedPathName = selectedAuto.getName();
          }
          if (!watchKey.pollEvents().isEmpty()) {
            updateAutoChooser();
            LOGGER.info("Updated autonomous chooser.");
          }
          if (!watchKey.reset())
            LOGGER.severe("File watch key invalid.");
          setName("Path Watcher: Waiting");
        }

        @Override
        public boolean runsWhenDisabled() {
          return true;
        }
      }.schedule();
      pathLoaderCommand.schedule();
    } catch (IOException exception) {
      LOGGER.log(Level.SEVERE, "Exception with path file watcher.", exception);
    }
    updateAutoChooser();
    recordInit();
  }

  private Command pathLoaderCommand = new CommandBase() {
    @Override
    public void execute() {
      var selectedAuto = autoChooser.getSelected();
      if (selectedAuto != null && !selectedAuto.getName().equals(loadedPathName)) {
        setName("LOADING PATH");
        Thread thread = new Thread(() -> {
          loadedPathTrajectory = PathPlanner.loadPath(pathExtensionRemover.apply(selectedAuto.getName()), 5.5, 50);
          setName("Path loader waiting");
        }, "Path Loader Thread");
        thread.start();
      }
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }.withName("Path loader");
  private Thread pathLoaderThread = new Thread();

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
    var selectedAuto = autoChooser.getSelected();
    if (loadedPathTrajectory != null) {
      // if (selectedAuto != null && !selectedAuto.getName().equals(loadedPathName))
      // loadedPathTrajectory =
      // PathPlanner.loadPath(pathExtensionRemover.apply(selectedAuto.getName()), 5.5,
      // 50);
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
      return new RunCommand(() -> {
      }).withName("No Autonomous Path");
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

  private void updateAutoChooser() {
    Arrays.stream(Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toFile().listFiles())
        .filter(file -> file.getName().endsWith(".path")).sorted(Comparator.comparingLong(File::lastModified))
        .forEachOrdered(file -> autoChooser.addOption(file.getName(), file));
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    if (Boolean.TRUE.equals(Errors.log().getWithDefault(outputFile::createNewFile, false))) {
      createPath(null, null, false).write(outputFile);
      autoChooser.setDefaultOption(outputFile.getName(), outputFile);
      LOGGER.log(Level.SEVERE, "Recorded path to {0}.", outputFile.getPath());
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
    PathPlannerUtil.Path path = new PathPlannerUtil.Path();
    for (int i = 0; i < pathPoints.size() - 2; i++)
      pathPoints.get(i).nextControl = pathPoints.get(i + 1).anchorPoint;
    for (int i = 1; i < pathPoints.size() - 1; i++)
      pathPoints.get(i).prevControl = pathPoints.get(i - 1).anchorPoint;
    path.waypoints = Optional.ofNullable(pathPoints.toArray(PathPlannerUtil.Path.Waypoint[]::new));
    path.maxVelocity = Optional.ofNullable(maxVelocity);
    path.maxAcceleration = Optional.ofNullable(maxAcceleration);
    path.isReversed = Optional.ofNullable(isReversed);
    pathPoints.clear();
    return path;
  }
}
