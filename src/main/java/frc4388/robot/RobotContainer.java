// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import java.util.Objects;
import java.util.logging.Logger;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.AutoConstants;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.commands.PathRecorder;
import frc4388.robot.commands.ExtenderIntakeCommands.ExtenderIntakeGroup;
import frc4388.robot.commands.shooter.TimedWaitUntilCommand;
import frc4388.robot.commands.shooter.TrackTarget;
import frc4388.robot.commands.shuffleboard.CommandSchedule;
import frc4388.robot.commands.shuffleboard.ShooterTuner;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Camera;
import frc4388.robot.subsystems.Claws;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.Extender;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Serializer;
import frc4388.robot.subsystems.Storage;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.VisionOdometry;
import frc4388.utility.controller.ButtonBox;
import frc4388.utility.controller.DeadbandedXboxController;

//TODO: Try using ConditionalCommand for subsystem default commands.
//TODO: Replace Path Recorder with Auto Chooser.
//TODO: Add POV button pad bindings as an example.
//XXX: Re-enable extender in autonomous.

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final Logger LOGGER = Logger.getLogger(RobotContainer.class.getSimpleName());

  /* Robot Map */
  public final RobotMap m_robotMap = new RobotMap();

  /* Subsystems */
  public final Camera m_robotCamera = new Camera("driver", 0, 160, 120, 40);
  public final Climber m_robotClimber = new Climber(m_robotMap.elbow);
  public final Claws m_robotClaws = new Claws(m_robotMap.leftClaw, m_robotMap.rightClaw);
  public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.frontLeft, m_robotMap.frontRight, m_robotMap.backLeft, m_robotMap.backRight, m_robotMap.gyro);
  public final Serializer m_robotSerializer = new Serializer(m_robotMap.serializerBelt);
  public final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor);
  public final Extender m_robotExtender = new Extender(m_robotMap.extenderMotor);
  public final Storage m_robotStorage = new Storage(m_robotMap.storageMotor);
  public final LED m_robotLED = new LED(m_robotMap.LEDController);
  public final BoomBoom m_robotBoomBoom = new BoomBoom(m_robotMap.shooterFalconLeft, m_robotMap.shooterFalconRight);
  public final Hood m_robotHood = new Hood(m_robotMap.angleAdjusterMotor);
  public final Turret m_robotTurret = new Turret(m_robotMap.shooterTurret);
  public final VisionOdometry m_robotVisionOdometry = new VisionOdometry(m_robotSwerveDrive, m_robotTurret);

  /* Dashboard Tools */
  private final PathRecorder m_pathChooser = new PathRecorder(m_robotSwerveDrive);
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private final ShooterTuner m_shooterTuner = new ShooterTuner(m_robotBoomBoom);
  private final CommandSchedule m_commandSchedule = new CommandSchedule(10, 5, false);

  /* Controllers */
  private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
  private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(OIConstants.BUTTON_BOX_ID);

  private static boolean softLimits = true;

  // control mode switching
  private enum ControlMode {
    SHOOTER, CLIMBER
  }

  private ControlMode currentControlMode = ControlMode.SHOOTER;

  // drive on off mode switching
  private enum DriveMode {
    ON, OFF
  }

  private DriveMode currentDriveMode = DriveMode.ON;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoChooser.addOption("threeBallPlus", threeBallPlus);

    SmartDashboard.putData("AutoChooser", autoChooser);

    Preferences.initString("Autonomous", "Three Ball");

    configureButtonBindings();
    /* Default Commands */
    // Swerve Drive with Input
    m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
      if (currentDriveMode == DriveMode.ON) {
        m_robotSwerveDrive.driveWithInput(getDriverController().getLeft(), getDriverController().getRight(), true);
      }
      if (currentDriveMode == DriveMode.OFF) {
        m_robotSwerveDrive.driveWithInput(0, 0, 0, 0, false);
      }
    }, m_robotSwerveDrive).withName("SwerveDrive DefaultCommand"));

    // Intake with Triggers
    m_robotIntake.setDefaultCommand(new RunCommand(() -> m_robotIntake.runWithTriggers(getOperatorController().getLeftTriggerAxis(), getOperatorController().getRightTriggerAxis()), m_robotIntake).withName("Intake DefaultCommand"));

    // TODO: Comment
    m_robotBoomBoom.setDefaultCommand(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.45), m_robotBoomBoom).withName("BoomBoom DefaultCommand"));

    // Serializer Manual
    m_robotSerializer.setDefaultCommand(new RunCommand(() -> m_robotSerializer.setSerializer(getOperatorController().getLeftTriggerAxis() * 0.8), m_robotSerializer).withName("Serializer DefaultCommand"));

    // Turret Manual
    m_robotTurret.setDefaultCommand(new RunCommand(() -> {
      if (currentControlMode == ControlMode.SHOOTER) {
        m_robotTurret.runTurretWithInput(getOperatorController().getLeftX());
      }
      if (currentControlMode == ControlMode.CLIMBER) {
        m_robotTurret.runTurretWithInput(0);
      }
    }, m_robotTurret));

    // Hood Manual
    m_robotHood.setDefaultCommand(new RunCommand(() -> {
      if (currentControlMode == ControlMode.SHOOTER) {
        m_robotHood.runHood(getOperatorController().getLeftY());
      }
      if (currentControlMode == ControlMode.CLIMBER) {
        m_robotHood.runHood(0);
      }
    }, m_robotHood));

    // Climber Manual
    m_robotClimber.setDefaultCommand(new RunCommand(() -> {
      if (currentControlMode == ControlMode.SHOOTER) {
        m_robotClimber.setMotors(0.0);
      }
      if (currentControlMode == ControlMode.CLIMBER) {
        m_robotClimber.setMotors(-getOperatorController().getRightY());
      }
    }, m_robotClimber));

    SmartDashboard.putData("Shooter Tuner", m_shooterTuner);
    SmartDashboard.putData("Shooter Tuner", m_shooterTuner);
    SmartDashboard.putData("Command Schedule", m_commandSchedule);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ! Driver Buttons

    // Start > Calibrate Odometry
    new JoystickButton(getDriverController(), XboxController.Button.kBack.value).whenPressed(() -> resetOdometry(new Pose2d(0, 0, new Rotation2d(0))));

    // Start > Calibrate Odometry
    new JoystickButton(getDriverController(), XboxController.Button.kStart.value).whenPressed(m_robotSwerveDrive::resetGyro);

    // Left Bumper > Shift Down
    new JoystickButton(getDriverController(), XboxController.Button.kLeftBumper.value).whenPressed(() -> m_robotSwerveDrive.highSpeed(false));

    // Right Bumper > Shift Up
    new JoystickButton(getDriverController(), XboxController.Button.kRightBumper.value).whenPressed(() -> m_robotSwerveDrive.highSpeed(true));

    new JoystickButton(getDriverController(), XboxController.Button.kA.value).whenPressed(new InstantCommand(() -> switchControlMode())).whenReleased(new InstantCommand(() -> switchControlMode()));

    new JoystickButton(getDriverController(), XboxController.Button.kB.value).whenPressed(new InstantCommand(() -> switchDriveMode())).whenReleased(new InstantCommand(() -> switchDriveMode()));

    // ! Operator Buttons

    // Right Bumper > Storage Out
    new JoystickButton(getOperatorController(), XboxController.Button.kRightBumper.value).whileHeld(new RunCommand(() -> m_robotStorage.runStorage(-StorageConstants.STORAGE_SPEED))).whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));

    // Left Bumper > Storage In
    new JoystickButton(getOperatorController(), XboxController.Button.kLeftBumper.value).whileHeld(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED))).whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));

    // B > Toggle claws
    new JoystickButton(getOperatorController(), XboxController.Button.kB.value).whenPressed(new InstantCommand(() -> m_robotClaws.toggleClaws(), m_robotClaws));

    // X > Toggles extender in and out
    new JoystickButton(getOperatorController(), XboxController.Button.kX.value).whenPressed(new ExtenderIntakeGroup(m_robotIntake, m_robotExtender));

    // A > Spit Out Ball
    new JoystickButton(getOperatorController(), XboxController.Button.kA.value).whileHeld(new RunCommand(() -> m_robotTurret.gotoMidpoint(), m_robotTurret)).whileHeld(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.25)));

    // Y > Full aim command
    // new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
    // .whileHeld(new Seek(m_robotSwerveDrive, m_robotBoomBoom, m_robotTurret, m_robotHood,
    // m_robotVisionOdometry));

    // ! Test Buttons
    // new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
    // .whenPressed(new Shoot(m_robotSwerveDrive, m_robotBoomBoom, m_robotTurret, m_robotHood,
    // m_robotVisionOdometry, false, false));

    // new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
    // .whenPressed(new RunCommandForTime(new RunCommand(() ->
    // m_robotTurret.runShooterRotatePID(-Math.atan2((219.25 / 2.00) - 10, (82.83 / 2.00) - 15.56)),
    // m_robotTurret), 1.0));
    new JoystickButton(getOperatorController(), XboxController.Button.kY.value).whileHeld(new TrackTarget(m_robotVisionOdometry, m_robotTurret, m_robotHood, m_robotBoomBoom)); // * aim with turret to target);

    // new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
    // .whileHeld(new RunCommand(() -> m_robotClaws.setOpen(true)));

    // new JoystickButton(getOperatorController(), XboxController.Button.kB.value)
    // .whileHeld(new RunCommand(() -> m_robotClaws.setOpen(false)));

    // new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
    // .whenPressed(new Shoot(m_robotSwerveDrive, m_robotBoomBoom, m_robotTurret, m_robotHood));

    // new JoystickButton(getOperatorController(), XboxController.Button.kA.value)
    // .whenPressed(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.25)))
    // .whenReleased(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.0)));

    // ! Button Box Buttons
    // Left Switch > Disables soft limits on press, release resets encoders (all for turret, hood,
    // climber, and extender)

    // SmartDashboard.putData("BB LEFT ON", new SequentialCommandGroup(
    // new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(false), m_robotTurret),
    // new InstantCommand(() -> m_robotTurret.calibrationSpeed = 0.3, m_robotTurret),

    // new InstantCommand(() -> m_robotHood.setHoodSoftLimits(false), m_robotHood),
    // new InstantCommand(() -> m_robotHood.calibrationSpeed = 0.3, m_robotHood),

    // new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(false), m_robotExtender)
    // ));

    // SmartDashboard.putData("BB LEFT OFF", new SequentialCommandGroup(
    // new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(true), m_robotTurret),
    // new InstantCommand(() -> m_robotTurret.calibrationSpeed = 1.0, m_robotTurret),

    // new InstantCommand(() -> m_robotHood.setHoodSoftLimits(true), m_robotHood),
    // new InstantCommand(() -> m_robotHood.calibrationSpeed = 1.0, m_robotHood),

    // new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(true), m_robotExtender),

    // new InstantCommand(() -> m_robotTurret.m_boomBoomRotateEncoder.setPosition(0), m_robotTurret),
    // new InstantCommand(() -> m_robotHood.m_angleEncoder.setPosition(0), m_robotHood),
    // new InstantCommand(() -> m_robotExtender.setEncoder(0), m_robotExtender),
    // new InstantCommand(() -> ExtenderIntakeGroup.setDirectionToOut(), m_robotIntake,
    // m_robotExtender),
    // new InstantCommand(() -> m_robotClimber.setEncoders(0), m_robotClimber)
    // ));

    // new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value)
    // .whenPressed(new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(false), m_robotTurret))
    // .whenPressed(new InstantCommand(() -> m_robotTurret.calibrationSpeed = 0.3, m_robotTurret))

    // .whenPressed(new InstantCommand(() -> m_robotHood.setHoodSoftLimits(false), m_robotHood))
    // .whenPressed(new InstantCommand(() -> m_robotHood.calibrationSpeed = 0.3, m_robotHood))

    // .whenPressed(new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(false),
    // m_robotExtender))

    // .whenReleased(new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(true), m_robotTurret))
    // .whenReleased(new InstantCommand(() -> m_robotTurret.calibrationSpeed = 1.0, m_robotTurret))

    // .whenReleased(new InstantCommand(() -> m_robotHood.setHoodSoftLimits(true), m_robotHood))
    // .whenReleased(new InstantCommand(() -> m_robotHood.calibrationSpeed = 1.0, m_robotHood))

    // .whenReleased(new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(true),
    // m_robotExtender))

    // .whenReleased(new InstantCommand(() -> m_robotTurret.m_boomBoomRotateEncoder.setPosition(0),
    // m_robotTurret))
    // .whenReleased(new InstantCommand(() -> m_robotHood.m_angleEncoder.setPosition(0), m_robotHood))
    // .whenReleased(new InstantCommand(() -> m_robotExtender.setEncoder(0), m_robotExtender))
    // .whenReleased(new InstantCommand(() -> ExtenderIntakeGroup.setDirectionToOut(), m_robotIntake,
    // m_robotExtender))
    // .whenReleased(new InstantCommand(() -> m_robotClimber.setEncoders(0), m_robotClimber));

    // Middle Switch > Climber and Shooter mode switching
    // new JoystickButton(getButtonBox(), ButtonBox.Button.kMiddleSwitch.value)
    // .whenPressed(new InstantCommand(() -> currentControlMode = ControlMode.CLIMBER))
    // .whenReleased(new InstantCommand(() -> currentControlMode = ControlMode.SHOOTER));
    // // new JoystickButton(getButtonBox(), ButtonBox.Button.kRightSwitch.value)
    // // .whenPressed(new InstantCommand(() -> currentControlMode = ControlMode.))
    // // .whenReleased(new InstantCommand(() -> currentControlMode = ControlMode.SHOOTER));

    // new JoystickButton(getButtonBox(), ButtonBox.Button.kRightSwitch.value)
    // .whileHeld(new InstantCommand(() -> currentDriveMode = DriveMode.OFF))
    // .whenReleased(new InstantCommand(() -> currentDriveMode = DriveMode.ON));

    // // Left Button > Extender In
    new JoystickButton(getDriverController(), XboxController.Button.kX.value).whileHeld(new RunCommand(() -> m_robotExtender.runExtender(1.0), m_robotExtender)).whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));

    // Left Button > Extender Out
    new JoystickButton(getDriverController(), XboxController.Button.kY.value).whileHeld(new RunCommand(() -> m_robotExtender.runExtender(-1.0), m_robotExtender)).whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));
  }

  private boolean isLockedOn() {
    return m_robotTurret.isLockedOn() || m_robotHood.isLockedOn() || m_robotBoomBoom.isLockedOn();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private final CommandGroupBase threeBallPlus = CommandGroupBase.sequence(
    new InstantCommand(() -> m_robotBoomBoom.runDrumShooterVelocityPID(8000), m_robotBoomBoom).withName("StartIdlingShooter"),
    new InstantCommand(() -> m_robotIntake.runAtOutput(-1), m_robotIntake).withName("StartRunningIntake"),
    // new RunExtender(m_robotExtender).withName("DeployExtender"),
    new InstantCommand(() -> m_robotSerializer.setSerializer(0.8), m_robotSerializer).withName("StartRunningSerializer"),
    // Get Second Ball
    new InstantCommand(() -> m_robotTurret.runShooterRotatePID(-180), m_robotTurret).withName("StartTurningShooter"),
    makePathingGroup(AutoConstants.PATH_MAX_VEL, AutoConstants.PATH_MAX_ACCEL, "JMove1").withName("JMove1"),
    // Shoot Preloaded and Second Ball
    makeTimeoutTrackShotGroup(AutoConstants.LOCK_ON_DURATION, AutoConstants.LOCK_ON_TIME_ALLOWANCE, AutoConstants.STORAGE_TIME_TWO_BALLS, "FirstSecondBall"),
    // Get Third Ball
    new InstantCommand(() -> m_robotBoomBoom.runDrumShooterVelocityPID(8000), m_robotBoomBoom).withName("StartIdlingShooter"),
    new InstantCommand(() -> m_robotTurret.runShooterRotatePID(-120), m_robotTurret).withName("StartTurningShooter"),
    makePathingGroup(AutoConstants.PATH_MAX_VEL, AutoConstants.PATH_MAX_ACCEL, "JMove2").withName("JMove2"),
    // Shoot Third Ball
    makeTimeoutTrackShotGroup(AutoConstants.LOCK_ON_DURATION, AutoConstants.LOCK_ON_TIME_ALLOWANCE, AutoConstants.STORAGE_TIME_ONE_BALL, "ThirdBall"),
    // Stop
    new InstantCommand(() -> m_robotIntake.runAtOutput(0), m_robotIntake).withName("StopRunningIntake"),
    new InstantCommand(() -> m_robotSerializer.setSerializer(0.0), m_robotSerializer).withName("StopRunningSerializer"),
    new InstantCommand(() -> m_robotStorage.runStorage(0.0)).withName("ThirdBallStopFeed")
  );

  private ParallelDeadlineGroup makeTimeoutTrackShotGroup(double lockOnDuration, double lockOnTimeAllowance, double storageRunTime, String name) {
    return CommandGroupBase.sequence(
      new TimedWaitUntilCommand(this::isLockedOn, lockOnDuration).withTimeout(lockOnTimeAllowance).withName(name + "LockOn"),
      new InstantCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED), m_robotStorage).withName(name + "Feed"),
      new WaitCommand(storageRunTime).withName(name + "ShootTimer"),
      new InstantCommand(() -> m_robotStorage.runStorage(0.0)).withName(name + "StopFeed")).deadlineWith(new TrackTarget(m_robotVisionOdometry, m_robotTurret, m_robotHood, m_robotBoomBoom).withName(name + "Track")
    );
  }

  /**
   * Generate autonomous
   * @param maxVel max velocity for the path (null to override default value of 5.0)
   * @param maxAccel max acceleration for the path (null to override default value of 5.0)
   * @param inputs strings (paths) or commands you want to run (in order)
   * @return array of commands
   */
  private SequentialCommandGroup makePathingGroup(Double maxVel, Double maxAccel, String input) {
    PIDController xController = SwerveDriveConstants.X_CONTROLLER;
    PIDController yController = SwerveDriveConstants.Y_CONTROLLER;
    ProfiledPIDController thetaController = SwerveDriveConstants.THETA_CONTROLLER;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(input, Objects.requireNonNullElse(maxVel, SwerveDriveConstants.PATH_MAX_VELOCITY), Objects.requireNonNullElse(maxAccel, SwerveDriveConstants.PATH_MAX_ACCELERATION));
    PathPlannerState initialState = pathTrajectory.getInitialState();
    Pose2d initialPosition = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotSwerveDrive.resetOdometry(initialPosition), m_robotSwerveDrive),
      new PPSwerveControllerCommand(pathTrajectory, m_robotSwerveDrive::getOdometry, m_robotSwerveDrive.m_kinematics, xController, yController, thetaController, m_robotSwerveDrive::setModuleStates, m_robotSwerveDrive),
      new InstantCommand(m_robotSwerveDrive::stopModules, m_robotSwerveDrive)
    );
  }

  public void switchControlMode() {
    currentControlMode = currentControlMode == ControlMode.SHOOTER ? ControlMode.CLIMBER : ControlMode.SHOOTER;
  }

  public void switchDriveMode() {
    currentDriveMode = currentDriveMode == DriveMode.ON ? DriveMode.OFF : DriveMode.ON;
  }

  public DeadbandedXboxController getDriverController() {
    return m_driverXbox;
  }

  public DeadbandedXboxController getOperatorController() {
    return m_operatorXbox;
  }

  public ButtonBox getButtonBox() {
    return m_buttonBox;
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
