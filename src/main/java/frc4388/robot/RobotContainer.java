// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import java.io.File;
import java.io.IOException;
import java.io.StringWriter;
import java.nio.file.FileSystems;
import java.nio.file.StandardWatchEventKinds;
import java.nio.file.WatchEvent;
import java.nio.file.WatchKey;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.logging.Logger;
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
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.commands.PathRecorder;
import frc4388.robot.commands.DriveCommands.DriveWithInputForTime;
import frc4388.robot.commands.ExtenderIntakeCommands.ExtenderIntakeGroup;
import frc4388.robot.commands.ShooterCommands.TrackTarget;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Claws;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.Extender;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.Serializer;
import frc4388.robot.subsystems.Storage;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.VisionOdometry;
import frc4388.utility.PathPlannerUtil;
import frc4388.utility.controller.ButtonBox;
import frc4388.utility.controller.DeadbandedRawXboxController;

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
  public final Climber m_robotClimber = new Climber(m_robotMap.elbow);
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

  /* Autonomous */
  private final PathRecorder m_pathChooser = new PathRecorder(m_robotSwerveDrive);
  // Controllers
  private final static XboxController m_driverXbox = new DeadbandedRawXboxController(OIConstants.XBOX_DRIVER_ID);
  private final static XboxController m_operatorXbox = new DeadbandedRawXboxController(OIConstants.XBOX_OPERATOR_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(OIConstants.BUTTON_BOX_ID);
  
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
      // Swerve Drive with Input
    m_robotSwerveDrive.setDefaultCommand(
        new RunCommand(() -> {
          if (RobotContainer.currentControlMode.equals(ControlMode.SHOOTER)) {
            m_robotSwerveDrive.driveWithInput( getDriverController().getLeftX(), 
                                               getDriverController().getLeftY(),
                                               getDriverController().getRightX(),
                                               getDriverController().getRightY(),
                                               true); }
          if (RobotContainer.currentControlMode.equals(ControlMode.CLIMBER)) {
            m_robotSwerveDrive.driveWithInput( 0, 
                                               0,
                                               0,
                                               0,
                                               true);
         }}
          , m_robotSwerveDrive).withName("Swerve driveWithInput defaultCommand"));

      // Intake with Triggers
    m_robotIntake.setDefaultCommand(
        new RunCommand(() -> m_robotIntake.runWithTriggers(
            getOperatorController().getLeftTriggerAxis(), 
            getOperatorController().getRightTriggerAxis()),
            m_robotIntake).withName("Intake runWithTriggers defaultCommand"));
    m_robotBoomBoom.setDefaultCommand(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.0), m_robotBoomBoom));

      // Serializer Manual
    m_robotSerializer.setDefaultCommand(
        new RunCommand(() -> m_robotSerializer.setSerializer(getOperatorController().getLeftTriggerAxis() * 0.8),//m_robotSerializer.setSerializerStateWithBeam(), 
        m_robotSerializer).withName("Serializer setSerializerStateWithBeam defaultCommand"));

      // Turret Manual
    m_robotTurret.setDefaultCommand(
       new RunCommand(() -> {
        if (RobotContainer.currentControlMode.equals(ControlMode.SHOOTER)) { m_robotTurret.runTurretWithInput(getOperatorController().getLeftX()); }
        if (RobotContainer.currentControlMode.equals(ControlMode.CLIMBER)) { m_robotTurret.runTurretWithInput(0); }
       }, m_robotTurret));

      // Hood Manual
    m_robotHood.setDefaultCommand(
       new RunCommand(() -> {
        if (RobotContainer.currentControlMode.equals(ControlMode.SHOOTER)) { m_robotHood.runHood(getOperatorController().getRightY()); }
        if (RobotContainer.currentControlMode.equals(ControlMode.CLIMBER)) { m_robotHood.runHood(0); }
       }, m_robotHood));

       //Climber Manual
    m_robotClimber.setDefaultCommand(
      new RunCommand(() -> {
        if (RobotContainer.currentControlMode.equals(ControlMode.SHOOTER)) { m_robotClimber.setMotors(0.0); }
        if (RobotContainer.currentControlMode.equals(ControlMode.CLIMBER)) { m_robotClimber.setMotors(getOperatorController().getRightY()); }
       }, m_robotClimber));
    
    m_robotBoomBoom.setDefaultCommand(
      new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.45), m_robotBoomBoom)
      );

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //! Driver Buttons

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



    //! Operator Buttons

      // Right Bumper > Storage Out
    new JoystickButton(getOperatorController(), XboxController.Button.kRightBumper.value)
        .whileHeld(new RunCommand(() -> m_robotStorage.runStorage(-StorageConstants.STORAGE_SPEED)))
        .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));

      // Left Bumper > Storage In
    new JoystickButton(getOperatorController(), XboxController.Button.kLeftBumper.value)
        .whileHeld(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED)))
        .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));

      // B > Toggle claws
    new JoystickButton(getOperatorController(), XboxController.Button.kB.value)
        .whenPressed(new InstantCommand(() -> m_robotClaws.toggleClaws(), m_robotClaws));

      // X > Toggles extender in and out
    new JoystickButton(getOperatorController(), XboxController.Button.kX.value)
        .whenPressed(new ExtenderIntakeGroup(m_robotIntake, m_robotExtender));

      // A > Spit Out Ball
    new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
        .whileHeld(new RunCommand(() -> m_robotTurret.gotoMidpoint(), m_robotTurret))
        .whileHeld(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.25)));

      // Y > Full aim command
    // new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
    //     .whileHeld(new Seek(m_robotSwerveDrive, m_robotBoomBoom, m_robotTurret, m_robotHood, m_robotVisionOdometry));
    

      //! Test Buttons
    // new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
    //  .whenPressed(new Shoot(m_robotSwerveDrive, m_robotBoomBoom, m_robotTurret, m_robotHood, m_robotVisionOdometry, false, false));
      
    new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
     .whileHeld(new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, false));

    // new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
    //  .whileHeld(new RunCommand(() -> m_robotClaws.setOpen(true)));

    // new JoystickButton(getOperatorController(), XboxController.Button.kB.value)
    //  .whileHeld(new RunCommand(() -> m_robotClaws.setOpen(false)));

    // new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
    //  .whenPressed(new Shoot(m_robotSwerveDrive, m_robotBoomBoom, m_robotTurret, m_robotHood));

    // new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
    //     .whenPressed(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.25)))
    //     .whenReleased(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.0)));



    //! Button Box Buttons
      // Left Switch > Disables soft limits on press, release resets encoders (all for turret, hood, climber, and extender)
    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value)
        .whenPressed(new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(false), m_robotTurret))
        .whenPressed(new InstantCommand(() -> m_robotTurret.calibrationSpeed = 0.3, m_robotTurret))
      
        .whenPressed(new InstantCommand(() -> m_robotHood.setHoodSoftLimits(false), m_robotHood))
        .whenPressed(new InstantCommand(() -> m_robotHood.calibrationSpeed = 0.3, m_robotHood))
      
        .whenPressed(new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(false), m_robotExtender))
      
        .whenReleased(new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(true), m_robotTurret))
        .whenReleased(new InstantCommand(() -> m_robotTurret.calibrationSpeed = 1.0, m_robotTurret))
      
        .whenReleased(new InstantCommand(() -> m_robotHood.setHoodSoftLimits(true), m_robotHood))
        .whenReleased(new InstantCommand(() -> m_robotHood.calibrationSpeed = 1.0, m_robotHood))

        .whenReleased(new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(true), m_robotExtender))

        .whenReleased(new InstantCommand(() -> m_robotTurret.m_boomBoomRotateEncoder.setPosition(0), m_robotTurret))
        .whenReleased(new InstantCommand(() -> m_robotHood.m_angleEncoder.setPosition(0), m_robotHood))
        .whenReleased(new InstantCommand(() -> m_robotExtender.setEncoder(0), m_robotExtender))
        .whenReleased(new InstantCommand(() -> ExtenderIntakeGroup.setDirectionToOut(), m_robotIntake, m_robotExtender))
        .whenReleased(new InstantCommand(() -> m_robotClimber.setEncoders(0), m_robotClimber));
    
      // Middle Switch > Climber and Shooter mode switching
    new JoystickButton(getButtonBox(), ButtonBox.Button.kMiddleSwitch.value)
        .whenPressed(new InstantCommand(() -> this.currentControlMode = ControlMode.CLIMBER))
        .whenReleased(new InstantCommand(() -> this.currentControlMode = ControlMode.SHOOTER));

      // Left Button > Extender In
    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftButton.value)
        .whileHeld(new RunCommand(() -> m_robotExtender.runExtender(-1.0), m_robotExtender))
        .whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));

      // Left Button > Extender Out
    new JoystickButton(getButtonBox(), ButtonBox.Button.kRightButton.value)
        .whileHeld(new RunCommand(() -> m_robotExtender.runExtender(1.0), m_robotExtender))
        .whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));
  }

  /**
   * Generate autonomous
   * @param maxVel max velocity for the path (null to override default value of 5.0)
   * @param maxAccel max acceleration for the path (null to override default value of 5.0)
   * @param inputs strings (paths) or commands you want to run (in order)
   * @return array of commands
   */
  public Command[] buildAuto(Double maxVel, Double maxAccel, Object... inputs) {
    maxVel = Objects.requireNonNullElse(maxVel, SwerveDriveConstants.PATH_MAX_VELOCITY);
    maxAccel = Objects.requireNonNullElse(maxAccel, SwerveDriveConstants.PATH_MAX_ACCELERATION);

    ArrayList<Command> commands = new ArrayList<Command>();

    PIDController xController = SwerveDriveConstants.X_CONTROLLER;
    PIDController yController = SwerveDriveConstants.Y_CONTROLLER;
    ProfiledPIDController thetaController = SwerveDriveConstants.THETA_CONTROLLER;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // parse input
    for (int i=0; i<inputs.length; i++) {
      if (inputs[i] instanceof String) {

        PathPlannerTrajectory traj = PathPlanner.loadPath(inputs[i].toString(), maxVel, maxAccel);
        PathPlannerState initState = traj.getInitialState();

        Pose2d initPose = new Pose2d(initState.poseMeters.getTranslation(), initState.holonomicRotation);

        commands.add(new InstantCommand(() -> m_robotSwerveDrive.resetOdometry(initPose), m_robotSwerveDrive));
        commands.add(new PPSwerveControllerCommand(
                          traj,
                          m_robotSwerveDrive::getOdometry,
                          m_robotSwerveDrive.m_kinematics,
                          xController,
                          yController,
                          thetaController,
                          m_robotSwerveDrive::setModuleStates,
                          m_robotSwerveDrive));
      }

      if (inputs[i] instanceof Command) {
        commands.add((Command) inputs[i]);
      }
    }
    
    commands.add(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(0, 0, 0, 0, true), m_robotSwerveDrive));
    Command[] ret = new Command[commands.size()];
    ret = commands.toArray(ret);
    return ret;
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

    // ! this will run each of the specified PathPlanner paths in sequence.
    // * return new SequentialCommandGroup(buildAuto(5.0, 5.0, "Path1", "Path2", "Path3"));

    // ! this will run each of the specified PathPlanner paths in sequence, while simultaneously running the intake throughout all the paths.
    // * return new ParallelCommandGroup(buildAuto(null,
    // *                                           null,
    // *                                           new SequentialCommandGroup(buildAuto(5.0, 5.0, "Path1", "Path2", "Path3")),
    // *                                           new RunCommand(() -> m_robotIntake.runAtOutput(0.5))));
    
    // return new SequentialCommandGroup(buildAuto(1.0, 1.0, new InstantCommand(() -> m_robotSwerveDrive.m_gyro.reset(), m_robotSwerveDrive),
    //                                                       // new InstantCommand(() -> this.resetOdometry(new Pose2d())),
    //                                                       new InstantCommand(() -> m_robotSwerveDrive.setModuleRotationsToAngle(0.0), m_robotSwerveDrive), 
    //                                                       "Diamond"));

    // * assume turret is already pointed towards target.
    // return new SequentialCommandGroup( new InstantCommand(() -> m_robotSwerveDrive.resetGyro(), m_robotSwerveDrive),
    //                                    new DriveWithInputForTime(m_robotSwerveDrive, new double[] {0.5, 0.5, 0.0, 0.0}, 1.0),
    //                                    new ParallelRaceGroup(
    //                                      new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, true),
    //                                      new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED), m_robotStorage)
    //                                    ));

    // return new RunCommandForTime(new RunCommand(() -> m_robotSwerveDrive.driveWithInput(0, 0, 0, true), m_robotSwerveDrive), 1.0, true);

    return new SequentialCommandGroup( new InstantCommand(() -> m_robotSwerveDrive.resetGyro(), m_robotSwerveDrive),
                                       new DriveWithInputForTime(m_robotSwerveDrive, new double[] {0.0, 0.3, 0.0, 0.0}, 1.0));//,
                                      //  new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, true),
                                      //  new ParallelCommandGroup(
                                      //    new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, true),
                                      //    new RunCommandForTime(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED), m_robotStorage), 1.0)
                                       //));

    // * aim with RotateUntilTarget
    // return new SequentialCommandGroup( new InstantCommand(() -> m_robotSwerveDrive.resetGyro(), m_robotSwerveDrive),
    //                                    new DriveWithInputForTime(m_robotSwerveDrive, new double[] {0.5, 0.5, 0.0, 0.0}, 1.0),
    //                                    new RotateUntilTarget(m_robotSwerveDrive, m_robotVisionOdometry, 0.5),
    //                                    new ParallelRaceGroup(
    //                                      new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, true),
    //                                      new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED), m_robotStorage)
    //                                    ));
  }

  public static XboxController getDriverController() {
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
}
