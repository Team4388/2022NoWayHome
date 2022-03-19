// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.diffplug.common.base.Errors;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import org.ejml.dense.row.decomposition.hessenberg.TridiagonalDecompositionHouseholderOrig_DDRM;
import org.opencv.core.Point;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.*;
import frc4388.robot.subsystems.Claws;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.Claws.ClawType;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.commands.CommandChooser;
import frc4388.robot.commands.ButtonBoxCommands.RunMiddleSwitch;
import frc4388.robot.commands.ButtonBoxCommands.RunTurretOrClimberAuto;
import frc4388.robot.commands.ClimberCommands.RunClaw;
import frc4388.robot.commands.ClimberCommands.RunClimberPath;
// import frc4388.robot.commands.ButtonBoxCommands.TurretManual;
import frc4388.robot.commands.ExtenderIntakeCommands.ExtenderIntakeGroup;
import frc4388.robot.commands.ShooterCommands.AimToCenter;
import frc4388.robot.commands.ShooterCommands.Shoot;
import frc4388.robot.commands.ShooterCommands.TrackTarget;
import frc4388.robot.commands.StorageCommands.ManageStorage;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.Extender;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Serializer;
import frc4388.robot.subsystems.Storage;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.Vision;
import frc4388.robot.subsystems.VisionOdometry;

import frc4388.utility.LEDPatterns;
import frc4388.utility.ListeningSendableChooser;
import frc4388.utility.PathPlannerUtil;
import frc4388.utility.Vector2D;
import frc4388.utility.PathPlannerUtil.Path.Waypoint;
import frc4388.utility.controller.ButtonBox;
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

  // RobotMap
  public final RobotMap m_robotMap = new RobotMap();

  /* Subsystems */
  public final Climber m_robotClimber = new Climber(m_robotMap.shoulder, m_robotMap.elbow, m_robotMap.gyro, false);
  public final Claws m_robotClaws = new Claws(m_robotMap.leftClaw, m_robotMap.rightClaw); 
  public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront, m_robotMap.leftBack, m_robotMap.rightFront, m_robotMap.rightBack, m_robotMap.gyro);
  public final Serializer m_robotSerializer = new Serializer(m_robotMap.serializerBelt, /*m_robotMap.serializerShooterBelt,*/ m_robotMap.serializerBeam);
  public final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor);
  public final Extender m_robotExtender = new Extender(m_robotMap.extenderMotor);
  
  public final Storage m_robotStorage = new Storage(m_robotMap.storageMotor);
  // private final LED m_robotLED = new LED(m_robotMap.LEDController);
  public final BoomBoom m_robotBoomBoom = new BoomBoom(m_robotMap.shooterFalconLeft, m_robotMap.shooterFalconRight);
  public final Hood m_robotHood = new Hood(m_robotMap.angleAdjusterMotor);
  public final Turret m_robotTurret = new Turret(m_robotMap.shooterTurret);
  public final VisionOdometry m_robotVisionOdometry = new VisionOdometry(m_robotSwerveDrive, m_robotTurret);

  private final WPI_TalonFX testShoulderMotor = new WPI_TalonFX(30);
  private final WPI_TalonFX testElbowMotor = new WPI_TalonFX(31);
  //public final Climber m_robotClimber = new Climber(testShoulderMotor, testElbowMotor);

  // Controllers
  private final XboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
  private final XboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(OIConstants.BUTTON_BOX_ID);

  // Autonomous
  private PathPlannerTrajectory loadedPathTrajectory = null;
  private final ListeningSendableChooser<File> autoChooser = new ListeningSendableChooser<>(this::loadPath);
  private final List<Waypoint> pathPoints = new ArrayList<>();
  private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private final NetworkTable recordingNetworkTable = networkTableInstance.getTable("Recording");

  private static final DateTimeFormatter RECORDING_FILE_NAME_FORMATTER = DateTimeFormatter
      .ofPattern("'Recording' yyyy-MM-dd HH-mm-ss.SSS'.path'");
  private static final Clock SYSTEM_CLOCK = Clock.system(ZoneId.systemDefault());
  private static final Path PATHPLANNER_DIRECTORY = Filesystem.getDeployDirectory().toPath().resolve("pathplanner");
  // Function that removes the ".path" from the end of a string.
  private static final Function<CharSequence, String> PATH_EXTENSION_REMOVER = ((Function<CharSequence, Matcher>) Pattern
      .compile(".path")::matcher).andThen(m -> m.replaceFirst(""));
  
  public static boolean softLimits = true;

  // control mode switching
  public static enum ControlMode { SHOOTER, CLIMBER };
  public static ControlMode currentControlMode = ControlMode.SHOOTER;

  // turret mode switching
  private enum TurretMode { MANUAL, AUTONOMOUS };
  private TurretMode currentTurretMode = TurretMode.MANUAL;

  // climber mode switching
  private enum ClimberMode { MANUAL, AUTONOMOUS };
  private ClimberMode currentClimberMode = ClimberMode.MANUAL;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    /* Default Commands */

    // IK command
    m_robotClimber.setDefaultCommand(
        new RunCommand(() -> m_robotClimber.controlJointsWithInput(getOperatorController().getLeftX(),
            getOperatorController().getRightY()), m_robotClimber).withName("Climber controlWithInput defaultCommand"));
    
    // Swerve Drive with Input
    m_robotSwerveDrive.setDefaultCommand(
        new RunCommand(() -> m_robotSwerveDrive.driveWithInput(
            getDriverController().getLeftX(),
            getDriverController().getLeftY(),
            getDriverController().getRightX(),
            getDriverController().getRightY(),
            true),
            m_robotSwerveDrive).withName("Swerve driveWithInput defaultCommand"));

    // Intake with Triggers
    m_robotIntake.setDefaultCommand(
        new RunCommand(() -> m_robotIntake.runWithTriggers(
            getOperatorController().getLeftTriggerAxis(), 
            getOperatorController().getRightTriggerAxis()),
            m_robotIntake).withName("Intake runWithTriggers defaultCommand"));
    
    // m_robotStorage.setDefaultCommand(
    //     new ManageStorage(m_robotStorage, 
    //                       m_robotBoomBoom, 
    //                       m_robotTurret).withName("Storage ManageStorage defaultCommand"));

    // Storage Management
    // m_robotStorage.setDefaultCommand(
    //     new RunCommand(() -> m_robotStorage.manageStorage(), 
    //     m_robotStorage).withName("Storage manageStorage defaultCommand"));

      // Serializer Management
    m_robotSerializer.setDefaultCommand(
        new RunCommand(() -> m_robotSerializer.setSerializer(getOperatorController().getLeftTriggerAxis() * 0.8),//m_robotSerializer.setSerializerStateWithBeam(), 
        m_robotSerializer).withName("Serializer setSerializerStateWithBeam defaultCommand"));
      // Turret Manual
    // m_robotTurret.setDefaultCommand(
    //     new RunCommand(() -> m_robotTurret.runTurretWithInput(getOperatorController().getLeftX()), 
    //     m_robotTurret).withName("Turret runTurretWithInput defaultCommand"));

    // m_robotTurret.setDefaultCommand(
    //    new RunCommand(() -> {
    //     if (this.currentControlMode.equals(ControlMode.SHOOTER)) { 
    //       if (this.currentTurretMode.equals(TurretMode.MANUAL)) { m_robotTurret.runTurretWithInput(getOperatorController().getLeftX()); }
    //     }
    //     if (this.currentControlMode.equals(ControlMode.CLIMBER)) { m_robotTurret.runTurretWithInput(0); }
    //    }, m_robotTurret));

    // m_robotHood.setDefaultCommand(
    //    new RunCommand(() -> {
    //     if (this.currentControlMode.equals(ControlMode.SHOOTER)) { m_robotHood.runHood(getOperatorController().getRightY()); }
    //     if (this.currentControlMode.equals(ControlMode.CLIMBER)) { m_robotHood.runHood(0); }
    //    }, m_robotHood));

    m_robotClimber.setDefaultCommand(
      // new RunCommand(() -> {
      //   if (this.currentMode.equals(Mode.SHOOTER)) { m_robotClimber.setMotors(0, 0); }
        /*if (this.currentMode.equals(Mode.CLIMBER)) {  */new RunCommand(() -> m_robotClimber.controlJointsWithInput(getOperatorController().getLeftX(),
          getOperatorController().getRightY()), m_robotClimber).withName("Climber controlWithInput defaultCommand")); //}
      // }, m_robotClimber));

       // m_robotTurret.setDefaultCommand(
    //     new AimToCenter(m_robotTurret, m_robotSwerveDrive, m_robotVisionOdometry));

    // continually sends updates to the Blinkin LED controller to keep the lights on
    //  m_robotLED.setDefaultCommand(new RunCommand(m_robotLED::updateLED, m_robotLED).withName("LED update defaultCommand"));

    //  autoInit();
    //  recordInit();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    /* Driver Buttons */

      // Start > Calibrate Odometry
    new JoystickButton(getDriverController(), XboxController.Button.kBack.value)
        .whenPressed(() -> resetOdometry(new Pose2d(0, 0, new Rotation2d(0))));
        // Start > Calibrate Odometry
    new JoystickButton(getDriverController(), XboxController.Button.kStart.value)
      .whenPressed(m_robotSwerveDrive::resetGyro);
      // Left Bumper > Shift Down
    new JoystickButton(getDriverController(), XboxController.Button.kLeftBumper.value)
         .whenPressed(() -> m_robotSwerveDrive.highSpeed(false));
      // Right Bumper > Shift Up
     new JoystickButton(getDriverController(), XboxController.Button.kRightBumper.value)
         .whenPressed(() -> m_robotSwerveDrive.highSpeed(true));

    /* Operator Buttons */

    new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
      .whileHeld(new RunCommand(() -> m_robotClaws.setOpen(true)));

    new JoystickButton(getOperatorController(), XboxController.Button.kB.value)
      .whileHeld(new RunCommand(() -> m_robotClaws.setOpen(false)));
  
    // new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
    //     .whenPressed(new InstantCommand(() -> m_robotVisionOdometry.setLEDs(false), m_robotVisionOdometry));

    new JoystickButton(getOperatorController(), XboxController.Button.kLeftBumper.value)
        .whenPressed(new RunCommand(() -> m_robotStorage.runStorage(0.9), m_robotStorage))
        .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0), m_robotStorage));

    new JoystickButton(getOperatorController(), XboxController.Button.kRightBumper.value)
        .whileHeld(new RunCommand(() -> m_robotStorage.runStorage(-0.9), m_robotStorage))
        .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0), m_robotStorage));

        //Toggles extender in and out
    new JoystickButton(getOperatorController(), XboxController.Button.kX.value)
        .whenPressed(new ExtenderIntakeGroup(m_robotIntake, m_robotExtender));

    new JoystickButton(getOperatorController(), XboxController.Button.kX.value)
        .whileHeld(new RunCommand(() -> m_robotTurret.gotoZero(), m_robotTurret))
        .whenReleased(new RunCommand(() -> m_robotTurret.m_boomBoomRotateMotor.set(0), m_robotTurret));

      // Right Bumper > Storage In
    // new JoystickButton(getOperatorController(), XboxController.Button.kRightBumper.value)
    //     .whileHeld(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED)))
    //     .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));

      // Left Bumper > Storage Out (note: necessary?)
    // new JoystickButton(getOperatorController(), XboxController.Button.kLeftBumper.value)
    //     .whileHeld(new RunCommand(() -> m_robotStorage.runStorage(-StorageConstants.STORAGE_SPEED)))
    //     .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));

      // A > Shoot with Odo
    /*new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
        .whenPressed(new Shoot(m_robotSwerveDrive, m_robotBoomBoom, m_robotTurret, m_robotHood));*/
      
      // new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
      //     .whileHeld(new RunCommand(() -> m_robotTurret.gotoMidpoint(), m_robotTurret));

       //B > Shoot with Lime
    new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
        .whileHeld(new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry))
        .whenReleased(new InstantCommand(() -> m_robotVisionOdometry.setLEDs(false)))
        .whenReleased(new RunCommand(() -> m_robotBoomBoom.runDrumShooterVelocityPID(0)));
        // .whileHeld%

    /* Button Box Buttons */

    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value)
      .whenPressed(new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(false), m_robotTurret))
      .whenPressed(new InstantCommand(() -> m_robotHood.setHoodSoftLimits(false), m_robotHood))
      .whenPressed(new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(false), m_robotExtender))
      .whenPressed(new InstantCommand(() -> m_robotClimber.setClimberSoftLimits(false), m_robotClimber))

      .whenReleased(new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(true), m_robotTurret))
      .whenReleased(new InstantCommand(() -> m_robotHood.setHoodSoftLimits(true), m_robotHood))
      .whenReleased(new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(true), m_robotExtender))
      .whenReleased(new InstantCommand(() -> m_robotClimber.setClimberSoftLimits(true), m_robotClimber))

      .whenReleased(new InstantCommand(() -> m_robotTurret.m_boomBoomRotateEncoder.setPosition(0), m_robotTurret))
      .whenReleased(new InstantCommand(() -> m_robotHood.m_angleEncoder.setPosition(0), m_robotHood))
      .whenReleased(new InstantCommand(() -> m_robotExtender.setEncoder(0), m_robotExtender))
      .whenReleased(new InstantCommand(() -> ExtenderIntakeGroup.setDirectionToOut(), m_robotIntake, m_robotExtender))
      .whenReleased(new InstantCommand(() -> m_robotClimber.setEncoders(0), m_robotClimber));
    
    new JoystickButton(getButtonBox(), ButtonBox.Button.kMiddleSwitch.value)

    //   .whenPressed(new InstantCommand(() -> m_robotTurret.setDefaultCommand(null)))
    //   .whenPressed(new InstantCommand(() -> m_robotHood.setDefaultCommand(null)))
    //   .whenPressed(new InstantCommand(() -> m_robotClimber.setDefaultCommand(
    //     new RunCommand(() -> m_robotClimber.setMotors(-getOperatorController().getLeftY(), -getOperatorController().getRightY()), m_robotClimber))))

    //   .whenReleased(new InstantCommand(() -> m_robotClimber.setDefaultCommand(null)))
    //   .whenReleased(new InstantCommand(() -> m_robotTurret.setDefaultCommand(
    //     new RunCommand(() -> m_robotTurret.runTurretWithInput(getOperatorController().getLeftX()), m_robotTurret))))
    //   .whenReleased(new InstantCommand(() -> m_robotHood.setDefaultCommand(
    //     new RunCommand(() -> m_robotHood.runHood(getOperatorController().getRightY()), m_robotHood))));

      .whenPressed(new InstantCommand(() -> this.currentControlMode = ControlMode.CLIMBER))
      .whenReleased(new InstantCommand(() -> this.currentControlMode = ControlMode.SHOOTER));
      // .whenReleased(EnableClimber()));
      
    new JoystickButton(getButtonBox(), ButtonBox.Button.kRightSwitch.value)

      .whenPressed(new InstantCommand(() -> {
        if (this.currentControlMode.equals(ControlMode.SHOOTER)) { this.currentTurretMode = TurretMode.AUTONOMOUS; } 
        if (this.currentControlMode.equals(ControlMode.CLIMBER)) { this.currentClimberMode = ClimberMode.AUTONOMOUS; }
      }))

      // * custom Command inside InstantCommand
      // .whenPressed(new InstantCommand(() -> {
      //   if (this.currentControlMode.equals(ControlMode.CLIMBER)) { new RunClimberPath(m_robotClimber, m_robotClaws, new Point[] {new Point()}); }
      //   if (this.currentControlMode.equals(ControlMode.SHOOTER)) { new AimToCenter(m_robotTurret, m_robotSwerveDrive, m_robotVisionOdometry); }
      // })
      //   .until(() -> this.currentClimberMode.equals(ClimberMode.MANUAL)))
      
      // * external RunTurretOrClimberAutos command, which runs either one (conditionally) based on currentControlMode
      // .whenPressed(new RunTurretOrClimberAuto(m_robotTurret, m_robotSwerveDrive, m_robotVisionOdometry, m_robotClimber, m_robotClaws))

      // * CommandChooser with BooleanSuppliers
      .whenPressed(new CommandChooser(new RunClimberPath(m_robotClimber, m_robotClaws, new Point[] {new Point()}), 
                                      new AimToCenter(m_robotTurret, m_robotSwerveDrive, m_robotVisionOdometry), 
                                      () -> this.currentControlMode.equals(ControlMode.CLIMBER),
                                      () -> this.currentControlMode.equals(ControlMode.SHOOTER)))

      .whenReleased(new InstantCommand(() -> {
        if (this.currentControlMode.equals(ControlMode.SHOOTER)) { this.currentTurretMode = TurretMode.MANUAL; }
        if (this.currentControlMode.equals(ControlMode.CLIMBER)) { this.currentClimberMode = ClimberMode.MANUAL; }
      }));

    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftButton.value)
    .whileHeld(new RunCommand(() -> m_robotExtender.runExtender(-1.0), m_robotExtender))
    .whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));

    new JoystickButton(getButtonBox(), ButtonBox.Button.kRightButton.value)
    .whileHeld(new RunCommand(() -> m_robotExtender.runExtender(1.0), m_robotExtender))
    .whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // if (loadedPathTrajectory != null) {
    //   PIDController xController = SwerveDriveConstants.X_CONTROLLER;
    //   PIDController yController = SwerveDriveConstants.Y_CONTROLLER;
    //   ProfiledPIDController thetaController = SwerveDriveConstants.THETA_CONTROLLER;
    //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //   PathPlannerState initialState = loadedPathTrajectory.getInitialState();
    //   Pose2d initialPosition = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
    //   return new SequentialCommandGroup(
    //       new InstantCommand(m_robotSwerveDrive.m_gyro::reset),
    //       new InstantCommand(() -> m_robotSwerveDrive.resetOdometry(initialPosition)),
    //       new PPSwerveControllerCommand(loadedPathTrajectory, m_robotSwerveDrive::getOdometry,
    //           m_robotSwerveDrive.m_kinematics, xController, yController, thetaController,
    //           m_robotSwerveDrive::setModuleStates, m_robotSwerveDrive),
    //       new InstantCommand(m_robotSwerveDrive::stopModules)).withName("Run Autonomous Path");
    // } else {
    //   LOGGER.severe("No auto selected.");
    //   return new RunCommand(() -> {
    //   }).withName("No Autonomous Path");
    // }
    return null;
  }

  public XboxController getDriverController() {
    return m_driverXbox;
  }

  public XboxController getOperatorController() {
    return m_operatorXbox;
  }

  public ButtonBox getButtonBox() {
    return m_buttonBox;
  }

  public static void setSoftLimits(boolean set) {
    softLimits = set;
  }

  /**
   * Get odometry.
   * 
   * @return Odometry
   */
  public Pose2d getOdometry() {
    return m_robotSwerveDrive.getOdometry();
  }

  /**
   * Set odometry to given pose.
   * 
   * @param pose Pose to set odometry to.
   */
  public void resetOdometry(Pose2d pose) {
    m_robotSwerveDrive.resetOdometry(pose);
  }

  /**
   * Creates a WatchKey for the path planner directory and registers it with the
   * WatchService.
   * Then creates a NotifierCommand that will update the auto chooser with the
   * latest path files.
   * Finally, adds the existing path files to the auto chooser
   */
  private void autoInit() {
    try {
      WatchKey watchKey = PATHPLANNER_DIRECTORY.register(FileSystems.getDefault().newWatchService(),
          StandardWatchEventKinds.ENTRY_CREATE, StandardWatchEventKinds.ENTRY_MODIFY,
          StandardWatchEventKinds.ENTRY_DELETE);
      // TODO: Store this and other commands as fields so they can be rescheduled.
      new NotifierCommand(() -> updateAutoChooser(watchKey), 0.5) {
        @Override
        public boolean runsWhenDisabled() {
          return true;
        }
      }.withName("Path Watcher").schedule();
    } catch (IOException exception) {
      LOGGER.log(Level.SEVERE, "Exception with path file watcher.", exception);
    }
    Arrays.stream(PATHPLANNER_DIRECTORY.toFile().listFiles())
        .filter(file -> file.getName().endsWith(".path")).sorted(Comparator.comparingLong(File::lastModified))
        .forEachOrdered(file -> autoChooser.addOption(file.getName(), file));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Creates a button on the SmartDashboard that will record the path of the
   * robot.
   */

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

  /**
   * Called when a file is created, modified, or deleted.
   * Adds newly created .path files to the SendableChooser.
   * Reloads the path if the currently selected file is modified.
   * 
   * @param watchKey The WatchKey that is being observed.
   */
  private void updateAutoChooser(WatchKey watchKey) {
    List<WatchEvent<?>> watchEvents = watchKey.pollEvents();
    if (!watchEvents.isEmpty()) {
      List<WatchEvent<?>> pathWatchEvents = watchEvents.stream()
          .filter(e -> e.kind().type().isAssignableFrom(Path.class)).collect(Collectors.toList());
      for (WatchEvent<?> pathWatchEvent : pathWatchEvents) {
        Path watchEventPath = (Path) pathWatchEvent.context();
        File watchEventFile = watchEventPath.toFile();
        String watchEventFileName = watchEventFile.getName();
        if (watchEventFileName.endsWith(".path")) {
          if (pathWatchEvent.kind().equals(StandardWatchEventKinds.ENTRY_CREATE)) {
            LOGGER.log(Level.WARNING, "PathPlanner file {0} created. Options added to SendableChooser.",
                watchEventFileName);
            autoChooser.addOption(watchEventFile.getName(), watchEventFile);
          } else if (pathWatchEvent.kind().equals(StandardWatchEventKinds.ENTRY_MODIFY)) {
            LOGGER.log(Level.WARNING, "PathPlanner file {0} modified.", watchEventFileName);
            if (watchEventFileName.equals(autoChooser.getSelected().getName())) {
              LOGGER.log(Level.SEVERE, "PathPlanner file {0} already selected. Reloading path.", watchEventFileName);
              loadPath(watchEventFileName);
            }
          } else if (pathWatchEvent.kind().equals(StandardWatchEventKinds.ENTRY_DELETE)) {
            LOGGER.log(Level.SEVERE,
                "PathPlanner file {0} deleted. Removing options from SendableChooser not yet implemented.",
                watchEventFileName);
          }
        }
      }
    }
    if (!watchKey.reset())
      LOGGER.severe("File watch key invalid.");
  }

  private void loadPath(String pathName) {
    LOGGER.warning("Loading path " + pathName);
    loadedPathTrajectory = null;
    loadedPathTrajectory = PathPlanner.loadPath(PATH_EXTENSION_REMOVER.apply(Objects.requireNonNullElse(pathName, "")),
        SwerveDriveConstants.PATH_MAX_VELOCITY, SwerveDriveConstants.PATH_MAX_ACCELERATION);
    LOGGER.info("Done loading");
  }

  private void saveRecording() {
    // IMPORTANT: Had to chown the pathplanner folder in order to save autos.
    File outputFile = PATHPLANNER_DIRECTORY
        .resolve(ZonedDateTime.now(SYSTEM_CLOCK).format(RECORDING_FILE_NAME_FORMATTER)).toFile();
    LOGGER.log(Level.WARNING, "Creating path {0}.", outputFile.getPath());
    if (!pathPoints.isEmpty() && Boolean.TRUE.equals(Errors.log().getWithDefault(outputFile::createNewFile, false))) {
      // TODO: Change to use measured maximum velocity and acceleration.
      var path = createPath(null, null, false);
      if (RobotBase.isReal())
        path.write(outputFile);
      StringWriter writer = new StringWriter();
      path.write(writer);
      recordingNetworkTable.getEntry(outputFile.getName()).setString(writer.toString());
      autoChooser.setDefaultOption(outputFile.getName(), outputFile);
      LOGGER.log(Level.INFO, "Recorded path to {0}.", outputFile.getPath());
    } else
      LOGGER.log(Level.SEVERE, "Unable to record path to {0}", outputFile.getPath());
  }

  public void recordPeriodic() {
    Translation2d position = m_robotSwerveDrive.m_poseEstimator.getEstimatedPosition().getTranslation();
    Rotation2d rotation = m_robotSwerveDrive.m_gyro.getRotation2d();
    // FIXME: Chassis speeds are created from joystick inputs and do not reflect
    // actual robot velocity.
    Translation2d velocity = new Translation2d(m_robotSwerveDrive.getChassisSpeeds()[0],
        m_robotSwerveDrive.getChassisSpeeds()[1]);
    Waypoint waypoint = new Waypoint(position, position, position, rotation.getDegrees(), false,
        SwerveDriveConstants.PATH_RECORD_VELOCITY ? velocity.getNorm() : null, false);
    pathPoints.add(waypoint);
  }

  public PathPlannerUtil.Path createPath(Double maxVelocity, Double maxAcceleration, Boolean isReversed) {
    // Remove points whose angles to neighboring points are less than 10 degrees
    // apart.
    int j = 0;
    for (int i = 1; i < pathPoints.size() - 1; i++) {
      var prev = pathPoints.get(j).anchorPoint.orElseThrow();
      var current = pathPoints.get(i).anchorPoint.orElseThrow();
      var next = pathPoints.get(i + 1).anchorPoint.orElseThrow();
      var fromPrevious = current.minus(prev);
      var toNext = next.minus(current);
      var angleFromPrevious = new Rotation2d(fromPrevious.getX(), fromPrevious.getY());
      var angleToNext = new Rotation2d(toNext.getX(), toNext.getY());
      if (Math.abs(angleFromPrevious.minus(angleToNext).getDegrees()) < SwerveDriveConstants.MIN_WAYPOINT_ANGLE
          || (next.getDistance(prev) < SwerveDriveConstants.MIN_WAYPOINT_DISTANCE
              && pathPoints.get(i).velOverride.map(v -> v < SwerveDriveConstants.MIN_WAYPOINT_VELOCITY).orElse(false)))
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
