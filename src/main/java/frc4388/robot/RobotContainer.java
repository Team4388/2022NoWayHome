// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;
import java.util.logging.Logger;

import com.diffplug.common.base.Errors;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.commands.PathRecorder;
import frc4388.robot.commands.RunCommandForTime;
import frc4388.robot.commands.ShooterTuner;
import frc4388.robot.commands.DriveCommands.DriveWithInputForTime;
import frc4388.robot.commands.ExtenderIntakeCommands.ExtenderIntakeGroup;
import frc4388.robot.commands.ShooterCommands.TrackTarget;
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
import frc4388.utility.Vector2D;
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
  public final Camera m_robotCamera = new Camera("driver", 0, 160, 120, 40);
  public final Climber m_robotClimber = new Climber(m_robotMap.elbow);
  public final Claws m_robotClaws = new Claws(m_robotMap.leftClaw, m_robotMap.rightClaw); 
  public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront, m_robotMap.leftBack, m_robotMap.rightFront, m_robotMap.rightBack, m_robotMap.gyro);
  public final Serializer m_robotSerializer = new Serializer(m_robotMap.serializerBelt, /*m_robotMap.serializerShooterBelt,*/ m_robotMap.serializerBeam);
  public final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor);
  public final Extender m_robotExtender = new Extender(m_robotMap.extenderMotor);
  
  public final Storage m_robotStorage = new Storage(m_robotMap.storageMotor);
  private final LED m_robotLED = new LED(m_robotMap.LEDController); // ! no LED makes aarav sad 
  public final BoomBoom m_robotBoomBoom = new BoomBoom(m_robotMap.shooterFalconLeft, m_robotMap.shooterFalconRight);
  public final Hood m_robotHood = new Hood(m_robotMap.angleAdjusterMotor);
  public final Turret m_robotTurret = new Turret(m_robotMap.shooterTurret);
  public final VisionOdometry m_robotVisionOdometry = new VisionOdometry(m_robotSwerveDrive, m_robotTurret);

  /* Autonomous */
  private final PathRecorder m_pathChooser = new PathRecorder(m_robotSwerveDrive);

  private final ShooterTuner m_shooterTuner = new ShooterTuner(m_robotBoomBoom);
  // Controllers
  private final static DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
  private final static DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);
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

  // drive on off mode switching
  private enum DriveMode { ON, OFF };
  private DriveMode currentDriveMode = DriveMode.ON;

  // private SendableChooser<SequentialCommandGroup> quickAutoChooser = new SendableChooser<>();
  public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

/**
 * SmartDash
 * - Limelight cam X
 * - Limit switches X
 * - Shooter RPM X
 * - Distance to target x
 * - target locked
 * - claws boolean
 * - field
 */

//  private SequentialCommandGroup makeTheWeirdGroup() {
//   SequentialCommandGroup weirdAutoShootingGroup = new SequentialCommandGroup(new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, true),
//   new ParallelCommandGroup(
//     new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, true),
//     new RunCommandForTime(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED), m_robotStorage), 2.0)
//   )); // * weird way of shooting, i think we should make a new TrackTarget with built-in Storage control instead.
//    return weirdAutoShootingGroup;
//  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoChooser.addOption("DriveOffLineAndShoot", driveOffLineAndShoot);
    autoChooser.addOption("OneBallAuto", oneBallAuto);
    autoChooser.setDefaultOption("TwoBallAuto", twoBallAuto);
    autoChooser.addOption("ThreeBallAuto", threeBallAuto);

    SmartDashboard.putData("AutoChooser", autoChooser);

    configureButtonBindings();
    /* Default Commands */
    // Swerve Drive with Input
    m_robotSwerveDrive.setDefaultCommand(
        new RunCommand(() -> {
          if (currentDriveMode == DriveMode.ON) {
            m_robotSwerveDrive.driveWithInput(getDriverController().getLeft(),
                getDriverController().getRight(), true);
          }
          if (currentDriveMode == DriveMode.OFF) {
            m_robotSwerveDrive.driveWithInput(0, 0, 0, 0, false);
          }
        }, m_robotSwerveDrive).withName("Swerve driveWithInput defaultCommand"));

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
        if (RobotContainer.currentControlMode.equals(ControlMode.SHOOTER)) { m_robotHood.runHood(getOperatorController().getLeftY()); }
        if (RobotContainer.currentControlMode.equals(ControlMode.CLIMBER)) { m_robotHood.runHood(0); }
       }, m_robotHood));

    //    //Climber Manual
    m_robotClimber.setDefaultCommand(
      new RunCommand(() -> {
        if (RobotContainer.currentControlMode.equals(ControlMode.SHOOTER)) { m_robotClimber.setMotors(0.0); }
        if (RobotContainer.currentControlMode.equals(ControlMode.CLIMBER)) { m_robotClimber.setMotors(-getOperatorController().getRightY()); }
       }, m_robotClimber));
    // m_robotClimber.setDefaultCommand(
    //   new RunCommand(() -> m_robotClimber.setMotors(-getOperatorController().getRightY()), m_robotClimber));
    
    m_robotBoomBoom.setDefaultCommand(
      new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.45), m_robotBoomBoom)
      );

    SmartDashboard.putData("Shooter Tuner", m_shooterTuner);
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

    new JoystickButton(getDriverController(), XboxController.Button.kA.value)
        .whenPressed(new InstantCommand(() -> switchControlMode()))
        .whenReleased(new InstantCommand(() -> switchControlMode()));

    new JoystickButton(getDriverController(), XboxController.Button.kB.value)
        .whenPressed(new InstantCommand(() -> switchDriveMode()))
        .whenReleased(new InstantCommand(() -> switchDriveMode()));

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
      
    // new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
    //  .whenPressed(new RunCommandForTime(new RunCommand(() -> m_robotTurret.runShooterRotatePID(-Math.atan2((219.25 / 2.00) - 10, (82.83 / 2.00) - 15.56)), m_robotTurret), 1.0));
    new JoystickButton(getOperatorController(), XboxController.Button.kY.value)
     .whileHeld(new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED)); // * aim with turret to target);

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

    // SmartDashboard.putData("BB LEFT ON", new SequentialCommandGroup(
    //   new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(false), m_robotTurret),
    //   new InstantCommand(() -> m_robotTurret.calibrationSpeed = 0.3, m_robotTurret),
    
    //   new InstantCommand(() -> m_robotHood.setHoodSoftLimits(false), m_robotHood),
    //   new InstantCommand(() -> m_robotHood.calibrationSpeed = 0.3, m_robotHood),
    
    //   new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(false), m_robotExtender)
    // ));
    
    // SmartDashboard.putData("BB LEFT OFF", new SequentialCommandGroup(
    //   new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(true), m_robotTurret),
    //   new InstantCommand(() -> m_robotTurret.calibrationSpeed = 1.0, m_robotTurret),
    
    //   new InstantCommand(() -> m_robotHood.setHoodSoftLimits(true), m_robotHood),
    //   new InstantCommand(() -> m_robotHood.calibrationSpeed = 1.0, m_robotHood),

    //   new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(true), m_robotExtender),

    //   new InstantCommand(() -> m_robotTurret.m_boomBoomRotateEncoder.setPosition(0), m_robotTurret),
    //   new InstantCommand(() -> m_robotHood.m_angleEncoder.setPosition(0), m_robotHood),
    //   new InstantCommand(() -> m_robotExtender.setEncoder(0), m_robotExtender),
    //   new InstantCommand(() -> ExtenderIntakeGroup.setDirectionToOut(), m_robotIntake, m_robotExtender),
    //   new InstantCommand(() -> m_robotClimber.setEncoders(0), m_robotClimber)
    // ));

    // new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value)
    //     .whenPressed(new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(false), m_robotTurret))
    //     .whenPressed(new InstantCommand(() -> m_robotTurret.calibrationSpeed = 0.3, m_robotTurret))
      
    //     .whenPressed(new InstantCommand(() -> m_robotHood.setHoodSoftLimits(false), m_robotHood))
    //     .whenPressed(new InstantCommand(() -> m_robotHood.calibrationSpeed = 0.3, m_robotHood))
      
    //     .whenPressed(new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(false), m_robotExtender))
      
    //     .whenReleased(new InstantCommand(() -> m_robotTurret.setTurretSoftLimits(true), m_robotTurret))
    //     .whenReleased(new InstantCommand(() -> m_robotTurret.calibrationSpeed = 1.0, m_robotTurret))
      
    //     .whenReleased(new InstantCommand(() -> m_robotHood.setHoodSoftLimits(true), m_robotHood))
    //     .whenReleased(new InstantCommand(() -> m_robotHood.calibrationSpeed = 1.0, m_robotHood))

    //     .whenReleased(new InstantCommand(() -> m_robotExtender.setExtenderSoftLimits(true), m_robotExtender))

    //     .whenReleased(new InstantCommand(() -> m_robotTurret.m_boomBoomRotateEncoder.setPosition(0), m_robotTurret))
    //     .whenReleased(new InstantCommand(() -> m_robotHood.m_angleEncoder.setPosition(0), m_robotHood))
    //     .whenReleased(new InstantCommand(() -> m_robotExtender.setEncoder(0), m_robotExtender))
    //     .whenReleased(new InstantCommand(() -> ExtenderIntakeGroup.setDirectionToOut(), m_robotIntake, m_robotExtender))
    //     .whenReleased(new InstantCommand(() -> m_robotClimber.setEncoders(0), m_robotClimber));
    
      // Middle Switch > Climber and Shooter mode switching
    // new JoystickButton(getButtonBox(), ButtonBox.Button.kMiddleSwitch.value)
    //     .whenPressed(new InstantCommand(() -> currentControlMode = ControlMode.CLIMBER))
    //     .whenReleased(new InstantCommand(() -> currentControlMode = ControlMode.SHOOTER));
    //     // new JoystickButton(getButtonBox(), ButtonBox.Button.kRightSwitch.value)
    //     // .whenPressed(new InstantCommand(() -> currentControlMode = ControlMode.))
    //     // .whenReleased(new InstantCommand(() -> currentControlMode = ControlMode.SHOOTER));
    
    // new JoystickButton(getButtonBox(), ButtonBox.Button.kRightSwitch.value)
    //     .whileHeld(new InstantCommand(() -> currentDriveMode = DriveMode.OFF))
    //     .whenReleased(new InstantCommand(() -> currentDriveMode = DriveMode.ON));

    //   // Left Button > Extender In
    new JoystickButton(getDriverController(), XboxController.Button.kX.value)
        .whileHeld(new RunCommand(() -> m_robotExtender.runExtender(1.0), m_robotExtender))
        .whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));

      // Left Button > Extender Out 
    new JoystickButton(getDriverController(), XboxController.Button.kY.value)
        .whileHeld(new RunCommand(() -> m_robotExtender.runExtender(-1.0), m_robotExtender))
        .whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));
  }

  /**
   * Generate autonomous
   * @param maxVel max velocity for the path (null to override default value of 5.0)
   * @param maxAccel max acceleration for the path (null to override default value of 5.0)
   * @param inputs strings (paths) or commands you want to run (in order)
   * @return array of commands
   */
  public SequentialCommandGroup buildAuto(Double maxVel, Double maxAccel, Object... inputs) {
    maxVel = Objects.requireNonNullElse(maxVel, SwerveDriveConstants.PATH_MAX_VELOCITY);
    maxAccel = Objects.requireNonNullElse(maxAccel, SwerveDriveConstants.PATH_MAX_ACCELERATION);

    ArrayList<Command> commands = new ArrayList<Command>();
    // commands.add(new InstantCommand(() -> m_robotSwerveDrive.resetGyro(), m_robotSwerveDrive));
    // commands.add(new InstantCommand(() -> m_robotSwerveDrive.m, m_robotSwerveDrive));

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
    
    commands.add(new InstantCommand(() -> m_robotSwerveDrive.stopModules(), m_robotSwerveDrive));
    Command[] ret = new Command[commands.size()];
    ret = commands.toArray(ret);
    SequentialCommandGroup seqCG = new SequentialCommandGroup(ret);
    return seqCG;
  }

  // ! ways to not coast
  // // * 1. try zero joystick input: new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(0.0, 0.0, 0.0, 0.0, false), m_robotSwerveDrive);
  // * 2. try opposite joystick input: new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(0.0, -1.0, 0.0, 0.0, false), m_robotSwerveDrive);
  // * 3a. try permanently setting drive motors to brake, not coast, in RobotMap.java, and ask the driver how it feels.
  // * 3b. try to only set the drive motors to brake if in auto mode.
  // * 4. try new InstantCommand(() -> m_robotSwerveDrive.stopModules(), m_robotSwerveDrive);
  
  // ? 1.0 input, 1 second: 134 inches
  // ? 0.75 input, 1 second: 48 inches
  // ! POSITIVE Y IS LEFT, POSITIVE X IS BACKWARDS

  double turretDistanceFromFront = 10.0; // * distance of turret from the front of the robot in inches. might need to be somewhat accurate.
  
  double distancePerSecond = 134.0; // * assuming emulated joystick input magnitude is 1.0
  double offset = 10.0; // * distance (in inches) from ball that we actually want to stop
  
  // ! ball positions are "unit tested"
  Vector2D firstBallPosition = new Vector2D(15.56 - (82.83 / 2.00), 11.21 - 162.00); // * position of first ball, relative to hub.
  Vector2D secondBallPosition = new Vector2D(-(40.44 * (Math.sqrt(2.00) / 2.00)) - ((82.83 - 7.58) * (Math.sqrt(2.00) / 2.00)) - (82.83 / 2.00), -(40.44 * (Math.sqrt(2.00) / 2.00)) + ((82.83 - 7.58) * (Math.sqrt(2.00) / 2.00)) - (219.25 / 2.00)); // * position of second ball, relative to hub.
  Vector2D firstToSecond = Vector2D.subtract(secondBallPosition, firstBallPosition); // * vector from first ball to second ball, used to calculate emulated joystick inputs.

  private SequentialCommandGroup shoot(double storageRunTime) {
    return new SequentialCommandGroup(
      new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED, true),
      new ParallelCommandGroup(
        new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED, true),
        new RunCommandForTime(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED), m_robotStorage), storageRunTime, true)
      )
    );
  }

  private ParallelRaceGroup shoot(double storageRunTime, double timeout) {
    return new SequentialCommandGroup(
      new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED, true),
      new ParallelCommandGroup(
        new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED, true),
        new RunCommandForTime(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED), m_robotStorage), storageRunTime, true)
      )
    ).withTimeout(timeout + storageRunTime);
  }

  SequentialCommandGroup weirdAutoShootingGroup = new SequentialCommandGroup(new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED, true),
                                                                               new ParallelCommandGroup(
                                                                                 new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED, true),
                                                                                 new RunCommandForTime(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED), m_robotStorage), 1.0, true)
                                                                               )); // * weird way of shooting, i think we should make a new TrackTarget with built-in Storage control instead.

  SequentialCommandGroup weirdAutoShootingGroup2 = new SequentialCommandGroup(new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED, true),
                                                                              new ParallelCommandGroup(
                                                                                new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED, true),
                                                                                new RunCommandForTime(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED), m_robotStorage), 2.3, true)
                                                                              )); // * weird way of shooting, i think we should make a new TrackTarget with built-in Storage control instead.

  SequentialCommandGroup weirdAutoShootingGroup3 = new SequentialCommandGroup(new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED, true),
                                                                              new ParallelCommandGroup(
                                                                                new TrackTarget(m_robotTurret, m_robotBoomBoom, m_robotHood, m_robotVisionOdometry, m_robotLED, true),
                                                                                new RunCommandForTime(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED), m_robotStorage), 4.0, true)
                                                                              )); // * weird way of shooting, i think we should make a new TrackTarget with built-in Storage control instead.                                                                         

  Command resetGyro = new InstantCommand(() -> m_robotSwerveDrive.resetGyro(), m_robotSwerveDrive);
  Command brakeDrive = new InstantCommand(() -> m_robotSwerveDrive.stopModules(), m_robotSwerveDrive);

  private Command brakeStorage(double storageRunTime) {
    return new RunCommandForTime(new RunCommand(() -> m_robotStorage.runStorage(0.0), m_robotIntake), storageRunTime, true);
  }

  SequentialCommandGroup extendThenAimTurret() {
    return new SequentialCommandGroup(
      new ExtenderIntakeGroup(m_robotIntake, m_robotExtender), 
      new RunCommandForTime(new RunCommand(() -> m_robotTurret.runShooterRotatePID(-175), m_robotTurret), 0.5, true) // TODO: optimize time
    );
  }
  
  ParallelDeadlineGroup idleDrumUntilShootingFirstBall() {
    return new ParallelDeadlineGroup(
      extendThenAimTurret(), 
      new RunCommand(() -> m_robotBoomBoom.runDrumShooterVelocityPID(9000), m_robotBoomBoom)
    );
  }

  private ParallelDeadlineGroup intakeWithPath1(double intakeRunTime) {
    return new ParallelDeadlineGroup(
      new RunCommandForTime(new RunCommand(() -> m_robotIntake.runAtOutput(-1.0), m_robotIntake), intakeRunTime, true), // TODO: optimize time
      new RunCommand(() -> m_robotSerializer.setSerializer(0.8), m_robotSerializer), 
      buildAuto(3.0, 3.0, "JMove1") // TODO: make faster?
    );
  }

  private ParallelDeadlineGroup intakeWithPath2(double intakeRunTime) {
    return new ParallelDeadlineGroup(
      new RunCommandForTime(new RunCommand(() -> m_robotIntake.runAtOutput(-1.0), m_robotIntake), intakeRunTime, true), // TODO: optimize time
      new RunCommand(() -> m_robotSerializer.setSerializer(0.8), m_robotSerializer), 
      buildAuto(3.0, 3.0, "JMove2") // TODO: make faster?
    );
  }

  ParallelCommandGroup intakeWithPathAndTrackTarget = new ParallelCommandGroup(intakeWithPath1(3.0), weirdAutoShootingGroup2);
  // ParallelCommandGroup intakeWithPath2AndTrackTarget = new ParallelCommandGroup(intakeWithPath2, weirdAutoShootingGroup3);

  ParallelDeadlineGroup intakeWithPath2AndIdleShooterAndAimTurret = new ParallelDeadlineGroup(
    intakeWithPath2(2.8),
    new RunCommand(() -> m_robotBoomBoom.runDrumShooterVelocityPID(8000), m_robotBoomBoom), 
    new RunCommandForTime(new RunCommand(() -> m_robotTurret.runShooterRotatePID(-120), m_robotTurret), 0.7, true)
  );

  // ! DRIVE OFF LINE AND SHOOT (ASSUMES ROBOT IS FACING DIRECTLY TOWARDS THE TARGET)
  SequentialCommandGroup driveOffLineAndShoot = new SequentialCommandGroup(
    resetGyro,
    new DriveWithInputForTime(m_robotSwerveDrive, new double[] {1.0, 0.0, 0.0, 0.0}, (5.0 * 12) / distancePerSecond), // * go backwards five feet
    brakeDrive,
    shoot(1.0),
    brakeStorage(0.1)
  );

  // ! ONE BALL AUTO (ASSUMES ROBOT IS FACING DIRECTLY TOWARDS THE TARGET)
  SequentialCommandGroup oneBallAuto = new SequentialCommandGroup(
    shoot(1.0),
    brakeStorage(0.1)
  );

  // ! TWO BALL AUTO (ASSUMES ROBOT IS FACING DIRECTLY TOWARDS THE FIRST BALL OUTSIDE THE TARMAC, BUMPERS FLUSH WITH THE EDGE)
  SequentialCommandGroup twoBallAuto = new SequentialCommandGroup(
    idleDrumUntilShootingFirstBall(),
    shoot(1.0, 4.0), // TODO: optimize time
    brakeStorage(0.1),
    intakeWithPath1(3.0), // * this line and the one underneath it can be replaced with intakeWithPathAndTrackTarget
    shoot(5.0), // TODO: optimize time
    brakeStorage(0.1)
  );

  // ! THREE BALL AUTO (ASSUMES ROBOT IS FACING DIRECTLY TOWARDS THE FIRST BALL OUTSIDE THE TARMAC, BUMPERS FLUSH WITH THE EDGE)
  SequentialCommandGroup threeBallAuto = new SequentialCommandGroup(
    idleDrumUntilShootingFirstBall(),
    shoot(0.8), // TODO: optimize time
    brakeStorage(0.1),
    intakeWithPathAndTrackTarget,
    // intakeWithPath1(3.0), // * this line and the one underneath it can be replaced with intakeWithPathAndTrackTarget
    shoot(0.8), // TODO: optimize time
    brakeStorage(0.1),
    intakeWithPath2AndIdleShooterAndAimTurret,
    shoot(4.0), // TODO: optimize time
    brakeStorage(0.1)
  );

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void switchControlMode() {
    if (currentControlMode == ControlMode.SHOOTER) {
      currentControlMode = ControlMode.CLIMBER;
    } else {
      currentControlMode = ControlMode.SHOOTER;
    }
  }

  public void switchDriveMode() {
    if (currentDriveMode == DriveMode.ON) {
      currentDriveMode = DriveMode.OFF;
    } else {
      currentDriveMode = DriveMode.ON;
    }
  }

  public static DeadbandedXboxController getDriverController() {
    return m_driverXbox;
  }

  public static DeadbandedXboxController getOperatorController() {
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
