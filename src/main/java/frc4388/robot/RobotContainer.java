// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.commands.AutonomousBuilder;
import frc4388.robot.commands.extender.DeployExtender;
import frc4388.robot.commands.extender.RetractExtender;
import frc4388.robot.commands.shooter.TrackTarget;
import frc4388.robot.commands.shuffleboard.CommandSchedule;
import frc4388.robot.commands.shuffleboard.PathRecorder;
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
import frc4388.utility.controller.XboxControllerPOV;
import frc4388.utility.shuffleboard.CachingSendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Robot Map */
  public final RobotMap m_robotMap = new RobotMap();

  /* Subsystems */
  public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.frontLeft, m_robotMap.frontRight, m_robotMap.backLeft, m_robotMap.backRight, m_robotMap.gyro);
  public final Extender m_robotExtender = new Extender(m_robotMap.extenderMotor);
  public final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor);
  public final Serializer m_robotSerializer = new Serializer(m_robotMap.serializerBelt);
  public final Storage m_robotStorage = new Storage(m_robotMap.storageMotor);
  public final BoomBoom m_robotBoomBoom = new BoomBoom(m_robotMap.shooterFalconLeft, m_robotMap.shooterFalconRight);
  public final Turret m_robotTurret = new Turret(m_robotMap.shooterTurret);
  public final Hood m_robotHood = new Hood(m_robotMap.angleAdjusterMotor);
  public final Claws m_robotClaws = new Claws(m_robotMap.leftClaw, m_robotMap.rightClaw);
  public final Climber m_robotClimber = new Climber(m_robotMap.elbow);
  public final LED m_robotLED = new LED(m_robotMap.LEDController);
  public final Camera m_robotCamera = new Camera("driver", 0, 160, 120, 40);
  public final VisionOdometry m_robotVisionOdometry = new VisionOdometry(m_robotSwerveDrive, m_robotTurret);

  /* Dashboard Tools */
  private final CachingSendableChooser<Command> m_autoChooser = new CachingSendableChooser<>();
  private final PathRecorder m_pathRecorder = new PathRecorder(m_robotSwerveDrive, m_autoChooser);
  private final ShooterTuner m_shooterTuner = new ShooterTuner(m_robotBoomBoom);
  private final CommandSchedule m_commandSchedule = new CommandSchedule(10, 5, false);

  /* Controllers */
  private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
  private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(OIConstants.BUTTON_BOX_ID);

  private boolean isClimberControlMode = false;
  private boolean driveControlEnabled = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    configureDefaultCommands();

    AutonomousBuilder autoBuilder = new AutonomousBuilder(this);
    m_autoChooser.addOption("OneBall", autoBuilder::buildOneBallCommand);
    m_autoChooser.addOption("TwoBall", autoBuilder::buildTwoBallCommand);
    m_autoChooser.setDefaultOption("ThreeBall", autoBuilder::buildThreeBallCommand);

    SmartDashboard.putData("Autonomous", m_autoChooser);
    SmartDashboard.putData("Path Recorder", m_pathRecorder);
    SmartDashboard.putData("Shooter Tuner", m_shooterTuner);
    SmartDashboard.putData("Command Schedule", m_commandSchedule);

    putRobotDashboardData();
  }

  /* Default Commands */
  private void configureDefaultCommands() {
    // Swerve Drive with Input
    m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
      if (driveControlEnabled) {
        m_robotSwerveDrive.driveWithInput(getDriverController().getLeft(), getDriverController().getRight(), true);
      } else {
        m_robotSwerveDrive.driveWithInput(0, 0, 0, 0, false);
      }
    }, m_robotSwerveDrive).withName("SwerveDrive DefaultCommand"));

    // Intake with Triggers
    m_robotIntake.setDefaultCommand(new RunCommand(() -> m_robotIntake.runWithTriggers(getOperatorController().getLeftTriggerAxis(), getOperatorController().getRightTriggerAxis()), m_robotIntake).withName("Intake DefaultCommand"));

    // Shooter Idle
    m_robotBoomBoom.setDefaultCommand(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.45), m_robotBoomBoom).withName("BoomBoom DefaultCommand"));

    // Serializer Manual
    m_robotSerializer.setDefaultCommand(new RunCommand(() -> m_robotSerializer.setSerializer(getOperatorController().getLeftTriggerAxis() * 0.8), m_robotSerializer).withName("Serializer DefaultCommand"));

    // Turret Manual
    m_robotTurret.setDefaultCommand(new RunCommand(() -> {
      if (!isClimberControlMode) {
        m_robotTurret.runTurretWithInput(getOperatorController().getLeftX());
      } else {
        m_robotTurret.runTurretWithInput(0);
      }
    }, m_robotTurret).withName("Turret DefaultCommand"));

    // Hood Manual
    m_robotHood.setDefaultCommand(new RunCommand(() -> {
      if (!isClimberControlMode) {
        m_robotHood.runHood(getOperatorController().getLeftY());
      } else {
        m_robotHood.runHood(0);
      }
    }, m_robotHood).withName("Hood DefaultCommand"));

    // Climber Manual
    m_robotClimber.setDefaultCommand(new RunCommand(() -> {
      if (!isClimberControlMode) {
        m_robotClimber.setMotors(0.0);
      } else {
        m_robotClimber.setMotors(-getOperatorController().getRightY());
      }
    }, m_robotClimber).withName("Climber DefaultCommand"));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureBoxButtonBindings();
  }

  private void configureDriverButtonBindings() {
    /* Left Bumper > Shift Down */
    new JoystickButton(getDriverController(), XboxController.Button.kLeftBumper.value)
        .whenPressed(() -> m_robotSwerveDrive.highSpeed(false));
    /* Right Bumper > Shift Up */
    new JoystickButton(getDriverController(), XboxController.Button.kRightBumper.value)
        .whenPressed(() -> m_robotSwerveDrive.highSpeed(false));
    /* Left Stick > Unbound */
    new JoystickButton(getDriverController(), XboxController.Button.kLeftStick.value)
        .whenPressed(new PrintCommand("Unbound"));
    /* Right Stick > Unbound */
    new JoystickButton(getDriverController(), XboxController.Button.kRightStick.value)
        .whenPressed(new PrintCommand("Unbound"));
    /* A > Unbound */
    new JoystickButton(getDriverController(), XboxController.Button.kA.value)
        .whenPressed(this::switchControlMode)
        .whenReleased(this::switchControlMode);
    /* B > Unbound */
    new JoystickButton(getDriverController(), XboxController.Button.kB.value)
        .whenPressed(this::switchDriveMode)
        .whenReleased(this::switchDriveMode);
    /* X > Unbound */
    new JoystickButton(getDriverController(), XboxController.Button.kX.value)
        .whileHeld(new RunCommand(() -> m_robotExtender.runExtender(1.0), m_robotExtender))
        .whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));
    /* Y > Unbound */
    new JoystickButton(getDriverController(), XboxController.Button.kY.value)
        .whileHeld(new RunCommand(() -> m_robotExtender.runExtender(-1.0), m_robotExtender))
        .whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));
    /* Back > Calibrate Odometry */
    new JoystickButton(getDriverController(), XboxController.Button.kBack.value)
        .whenPressed(() -> resetOdometry(new Pose2d(0, 0, new Rotation2d(0))));
    /* Start > Calibrate Odometry */
    new JoystickButton(getDriverController(), XboxController.Button.kStart.value)
        .whenPressed(m_robotSwerveDrive::resetGyro);

    /* D-pad Up > Unbound */
    new POVButton(getDriverController(), 0).whenPressed(new PrintCommand("Unbound"));
    /* D-pad Right > Unbound */
    new POVButton(getDriverController(), 90).whenPressed(new PrintCommand("Unbound"));
    /* D-pad Down > Unbound */
    new POVButton(getDriverController(), 180).whenPressed(new PrintCommand("Unbound"));
    /* D-pad Left > Unbound */
    new POVButton(getDriverController(), 270).whenPressed(new PrintCommand("Unbound"));
  }

  private void configureOperatorButtonBindings() {
    /* Left Bumper > Storage In */
    new JoystickButton(getDriverController(), XboxController.Button.kLeftBumper.value).whileHeld(new RunCommand(() -> m_robotStorage.runStorage(StorageConstants.STORAGE_SPEED)))
        .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));
    /* Right Bumper > Storage Out */
    new JoystickButton(getDriverController(), XboxController.Button.kRightBumper.value).whileHeld(new RunCommand(() -> m_robotStorage.runStorage(-StorageConstants.STORAGE_SPEED)))
        .whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));
    /* Left Stick > Unbound */
    new JoystickButton(getDriverController(), XboxController.Button.kLeftStick.value).whenPressed(new PrintCommand("Unbound"));
    /* Right Stick > Unbound */
    new JoystickButton(getDriverController(), XboxController.Button.kRightStick.value).whenPressed(new PrintCommand("Unbound"));
    /* A > Spit Out Ball */
    new JoystickButton(getDriverController(), XboxController.Button.kA.value).whileHeld(new RunCommand(m_robotTurret::gotoMidpoint, m_robotTurret))
        .whileHeld(new RunCommand(() -> m_robotBoomBoom.runDrumShooter(0.25)));
    /* B > Toggle Claws */
    new JoystickButton(getDriverController(), XboxController.Button.kB.value).whenPressed(m_robotClaws::toggleClaws, m_robotClaws);
    /* X > Toggle Extender Deployment */
    new JoystickButton(getDriverController(), XboxController.Button.kX.value).whenPressed(new ConditionalCommand(new DeployExtender(m_robotExtender, m_robotIntake), new RetractExtender(m_robotExtender), m_robotExtender::isRetracted));
    /* Y > Track Target */
    new JoystickButton(getDriverController(), XboxController.Button.kY.value).whileHeld(new TrackTarget(m_robotVisionOdometry, m_robotTurret, m_robotHood, m_robotBoomBoom));
    /* Back > Unbound */
    new JoystickButton(getDriverController(), XboxController.Button.kBack.value).whenPressed(new PrintCommand("Unbound"));
    /* Start > Unbound */
    new JoystickButton(getDriverController(), XboxController.Button.kStart.value).whenPressed(new PrintCommand("Unbound"));

    /* D-pad Up > Unbound */
    new POVButton(getDriverController(), 0).whenPressed(new PrintCommand("Unbound"));
    /* D-pad Right > Unbound */
    new POVButton(getDriverController(), 90).whenPressed(new PrintCommand("Unbound"));
    /* D-pad Down > Unbound */
    new POVButton(getDriverController(), 180).whenPressed(new PrintCommand("Unbound"));
    /* D-pad Left > Unbound */
    new POVButton(getDriverController(), 270).whenPressed(new PrintCommand("Unbound"));
  }

  private void configureBoxButtonBindings() {
    /* Right Switch > Unbound */
    new JoystickButton(getButtonBox(), ButtonBox.Button.kRightSwitch.value).whenPressed(new PrintCommand("Unbound"));
    /* Middle Switch > Climber and Shooter mode switching */
    new JoystickButton(getButtonBox(), ButtonBox.Button.kMiddleSwitch.value).whenPressed(() -> isClimberControlMode = true)
        .whenReleased(() -> isClimberControlMode = false);
    /* Left Switch > Disables soft limits on press, release resets encoders (all for turret, hood, climber, and extender) */
    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value).whenPressed(() -> m_robotTurret.setTurretSoftLimits(false), m_robotTurret)
          .whenPressed(() -> m_robotTurret.calibrationSpeed = 0.3, m_robotTurret);

    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value).whenPressed(() -> m_robotHood.setHoodSoftLimits(false), m_robotHood)
          .whenPressed(() -> m_robotHood.calibrationSpeed = 0.3, m_robotHood);

    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value).whenPressed(() -> m_robotExtender.setExtenderSoftLimits(false), m_robotExtender);

    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value).whenReleased(() -> m_robotTurret.setTurretSoftLimits(true), m_robotTurret)
        .whenReleased(() -> m_robotTurret.calibrationSpeed = 1.0, m_robotTurret);

    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value).whenReleased(() -> m_robotHood.setHoodSoftLimits(true), m_robotHood)
        .whenReleased(() -> m_robotHood.calibrationSpeed = 1.0, m_robotHood);

    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value).whenReleased(() -> m_robotExtender.setExtenderSoftLimits(true), m_robotExtender);

    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftSwitch.value).whenReleased(() -> m_robotTurret.m_boomBoomRotateEncoder.setPosition(0), m_robotTurret)
        .whenReleased(() -> m_robotHood.m_angleEncoder.setPosition(0), m_robotHood)
        .whenReleased(() -> m_robotExtender.setEncoder(0), m_robotExtender)
        .whenReleased(() -> m_robotClimber.setEncoders(0), m_robotClimber);
    /* Right Button > Extender Out */
    new JoystickButton(getButtonBox(), ButtonBox.Button.kRightButton.value).whileHeld(new RunCommand(() -> m_robotExtender.runExtender(1.0), m_robotExtender))
        .whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));
    /* Left Button > Extender In */
    new JoystickButton(getButtonBox(), ButtonBox.Button.kLeftButton.value).whileHeld(new RunCommand(() -> m_robotExtender.runExtender(-1.0), m_robotExtender))
        .whenReleased(new RunCommand(() -> m_robotExtender.runExtender(0.0), m_robotExtender));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public boolean isLockedOn() {
    return m_robotTurret.isLockedOn() || m_robotHood.isLockedOn() || m_robotBoomBoom.isLockedOn();
  }

  public void switchControlMode() {
    isClimberControlMode = !isClimberControlMode;
  }

  public void switchDriveMode() {
    driveControlEnabled = !driveControlEnabled;
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

  private void putRobotDashboardData() {
    putData("Values", builder -> {
      builder.addDoubleProperty("Match Time", DriverStation::getMatchTime, null);
      builder.addDoubleProperty("Claws", m_robotClaws.m_rightClaw::get, null);
      builder.addDoubleProperty("Arm", m_robotMap.elbow::get, null);
      builder.addDoubleProperty("Intake", m_robotMap.intakeMotor::get, null);
      builder.addDoubleProperty("Serializer", m_robotMap.serializerBelt::get, null);
      builder.addDoubleProperty("Storage", m_robotMap.storageMotor::get, null);
      builder.addDoubleProperty("Drum", m_robotMap.shooterFalconRight::get, null);
      builder.addDoubleProperty("Angle", m_robotMap.shooterTurret::get, null);
      builder.addDoubleProperty("Hood", m_robotMap.angleAdjusterMotor::get, null);
      builder.addBooleanProperty("Shooter Safety", this::isLockedOn, null);
    });
    putData("Field", m_robotSwerveDrive.m_field);
    putData("PDP", new PowerDistribution() {
      @Override
      public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("PowerDistributionPanel");
      }
    });
    putData("Extender", new NTSendable() {
      @Override
      public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        builder.getEntry(".instance").setDouble(0);
        builder.addStringProperty("default", () -> "Retracted", null);
        builder.addStringArrayProperty("options", () -> new String[] {"Retracted", "Extended"}, null);
        builder.addStringProperty("active", () -> m_robotExtender.getPosition() <= 0 ? "Retracted" : "Extended", null);
        builder.addStringProperty("selected", null, null);
      }
    });
    putData("Drivebase", m_robotSwerveDrive);
    putData("Gyro", m_robotSwerveDrive.m_gyro);
    putData("Drive Speed", new NTSendable() {
      @Override
      public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        builder.getEntry(".instance").setDouble(0);
        builder.addStringProperty("default", () -> "Low", null);
        builder.addStringArrayProperty("options", () -> new String[] {"Low", "High"}, null);
        builder.addStringProperty("active", () -> m_robotSwerveDrive.speedAdjust == SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW ? "Low" : "High", null);
        builder.addStringProperty("selected", null, val -> m_robotSwerveDrive.speedAdjust = val.equals("Low") ? SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW : SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_FAST);
      }
    });
    putData("Accelerometer", new NTSendable() {
      @Override
      public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("3AxisAccelerometer");
        NetworkTableEntry entryX = builder.getEntry("X");
        NetworkTableEntry entryY = builder.getEntry("Y");
        NetworkTableEntry entryZ = builder.getEntry("Z");
        builder.setUpdateTable(() -> {
          short[] data = new short[3];
          m_robotSwerveDrive.m_gyro.getBiasedAccelerometer(data);
          entryX.setDouble(data[0]);
          entryY.setDouble(data[1]);
          entryZ.setDouble(data[2]);
        });
      }
    });
    putData("Drive Camera", m_robotCamera);
  }

  private final Map<String, Sendable> m_tablesToData = new HashMap<>();
  private final NetworkTable m_networkTable = NetworkTableInstance.getDefault().getTable("Robot");

  private synchronized void putData(String key, Sendable data) {
    Sendable sddata = m_tablesToData.get(key);
    if (sddata == null || sddata != data) {
      m_tablesToData.put(key, data);
      NetworkTable dataTable = m_networkTable.getSubTable(key);
      SendableBuilderImpl builder = new SendableBuilderImpl();
      builder.setTable(dataTable);
      SendableRegistry.publish(data, builder);
      builder.startListeners();
      dataTable.getEntry(".name").setString(key);
    }
  }

  public synchronized void updateValues() {
    m_tablesToData.values().forEach(SendableRegistry::update);
  }
}
