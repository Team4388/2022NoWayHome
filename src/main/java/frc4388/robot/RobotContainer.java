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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.commands.AimToCenter;
import frc4388.robot.commands.PathRecorder;
import frc4388.robot.commands.Shoot;
import frc4388.robot.commands.TrackTarget;
import frc4388.robot.subsystems.BoomBoom;
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
  public final RobotMap m_robotMap = new RobotMap();

  // Subsystems 
  public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront, m_robotMap.leftBack, m_robotMap.rightFront, m_robotMap.rightBack, m_robotMap.gyro);
  public final Serializer m_robotSerializer = new Serializer(m_robotMap.serializerBelt, /*m_robotMap.serializerShooterBelt,*/ m_robotMap.serializerBeam);
  public final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor, m_robotMap.extenderMotor, m_robotSerializer);
  public final Storage m_robotStorage = new Storage(m_robotMap.storageMotor);
  // private final LED m_robotLED = new LED(m_robotMap.LEDController);
  public final BoomBoom m_robotBoomBoom = new BoomBoom(m_robotMap.shooterFalconLeft, m_robotMap.shooterFalconRight);
  public final Hood m_robotHood = new Hood(m_robotMap.angleAdjusterMotor);
  public final Turret m_robotTurret = new Turret(m_robotMap.shooterTurret);
  // private final VisionOdometry m_robotVisionOdometry = new VisionOdometry(m_robotSwerveDrive, m_robotTurret);*/
  
  /* Controllers */
  private final XboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
  private final XboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);

  /* Autonomous */
  private final PathRecorder m_pathChooser = new PathRecorder(m_robotSwerveDrive);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    /* Default Commands */

      // Swerve Drive with Input
    m_robotSwerveDrive.setDefaultCommand(
        new RunCommand(() -> m_robotSwerveDrive.driveWithInput(
            getDriverController().getLeftX(),
            getDriverController().getLeftY(),
            //getDriverController().getRightX(),
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
      // Storage Management
    /*m_robotStorage.setDefaultCommand(
        new RunCommand(() -> m_robotStorage.manageStorage(), 
        m_robotStorage).withName("Storage manageStorage defaultCommand"));*/
      // Serializer Management
    // m_robotSerializer.setDefaultCommand(
    //     new RunCommand(() -> m_robotSerializer.setSerializer(0.8),//m_robotSerializer.setSerializerStateWithBeam(), 
    //     m_robotSerializer).withName("Serializer setSerializerStateWithBeam defaultCommand"));
      // Turret Manual
    m_robotTurret.setDefaultCommand(
        new RunCommand(() -> m_robotTurret.runTurretWithInput(getOperatorController().getLeftX()), 
        m_robotTurret).withName("Turret runTurretWithInput defaultCommand"));
    m_robotHood.setDefaultCommand(
       new RunCommand(() -> m_robotHood.runHood(getOperatorController().getRightY() * 0.1), m_robotHood));
    // m_robotTurret.setDefaultCommand(
    //     new AimToCenter(m_robotTurret, m_robotSwerveDrive, m_robotVisionOdometry));

    // continually sends updates to the Blinkin LED controller to keep the lights on
    
    // m_robotLED.setDefaultCommand(new RunCommand(m_robotLED::updateLED, m_robotLED).withName("LED update defaultCommand"));
    // Creates a button on the SmartDashboard that will record the path of the robot.
    SmartDashboard.putData("Path Recording", m_pathChooser);
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

    // new JoystickButton(getDriverController(), XboxController.Button.kA.value)
    //     .whenPressed(() -> resetOdometry(new Pose2d(0, 0, new Rotation2d(0))));

    new JoystickButton(getDriverController(), XboxController.Button.kX.value) //Temp
        .whenPressed(() -> m_robotMap.leftFront.reset())
        .whenPressed(() -> m_robotMap.rightFront.reset())
        .whenPressed(() -> m_robotMap.leftBack.reset())
        .whenPressed(() -> m_robotMap.rightBack.reset());

    /* Operator Buttons */

      // X > Extend Intake
    /*new JoystickButton(getOperatorController(), XboxController.Button.kX.value)
        .whenPressed(() -> m_robotIntake.runExtender(true));
      // Y > Retract Intake
    new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
        .whenPressed(() -> m_robotIntake.runExtender(false));*/
    
    // new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
    //     .whenPressed(new InstantCommand(() -> m_robotTurret.m_boomBoomRotateEncoder.setPosition(0)));

    new JoystickButton(getOperatorController(), XboxController.Button.kB.value)
        .whenPressed(new InstantCommand(() -> m_robotHood.m_angleEncoder.setPosition(0)));    
      
    new JoystickButton(getOperatorController(), XboxController.Button.kX.value)
        .whileHeld(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.3)))
        .whenReleased(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.0)));

    new JoystickButton(getOperatorController(), XboxController.Button.kLeftBumper.value)
        .whileHeld(new RunCommand(() -> m_robotStorage.runStorage(-0.75), m_robotStorage))
        .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0), m_robotStorage));

    new JoystickButton(getOperatorController(), XboxController.Button.kRightBumper.value)
        .whileHeld(new RunCommand(() -> m_robotStorage.runStorage(0.75), m_robotStorage))
        .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0), m_robotStorage));
      // Right Bumper > Storage In
    // new JoystickButton(getOperatorController(), XboxController.Button.kRightBumper.value)
    //     .whileHeld(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED)))
    //     .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));
    //   // Left Bumper > Storage Out (note: neccessary?)
    // new JoystickButton(getOperatorController(), XboxController.Button.kLeftBumper.value)
    //     .whileHeld(new RunCommand(() -> m_robotStorage.runStorage(-StorageConstants.STORAGE_SPEED)))
    //     .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));
      // A > Shoot with Odo
    /*new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
        .whenPressed(new Shoot(m_robotSwerveDrive, m_robotBoomBoom, m_robotTurret, m_robotHood));
      // B > Shoot with Lime
    new JoystickButton(getOperatorController(), XboxController.Button.kB.value)
        .whenPressed(new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotSwerveDrive, m_robotVisionOdometry));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (m_pathChooser.getPath() != null) {
      PIDController xController = SwerveDriveConstants.X_CONTROLLER;
      PIDController yController = SwerveDriveConstants.Y_CONTROLLER;
      ProfiledPIDController thetaController = SwerveDriveConstants.THETA_CONTROLLER;
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      PathPlannerState initialState = m_pathChooser.getPath().getInitialState();
      Pose2d initialPosition = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
      return new SequentialCommandGroup(
          new InstantCommand(m_robotSwerveDrive.m_gyro::reset),
          new InstantCommand(() -> m_robotSwerveDrive.resetOdometry(initialPosition)),
          new PPSwerveControllerCommand(m_pathChooser.getPath(), m_robotSwerveDrive::getOdometry,
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

  public XboxController getOperatorController() {
    return m_operatorXbox;
  }
}
