package frc4388.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc4388.robot.RobotContainer;
import frc4388.robot.Constants.AutoConstants;
import frc4388.robot.Constants.StorageConstants;
import frc4388.robot.commands.TimedWaitUntilCommand;
import frc4388.robot.commands.extender.DeployExtender;
import frc4388.robot.commands.shooter.TrackTarget;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Extender;
import frc4388.robot.subsystems.Hood;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.Serializer;
import frc4388.robot.subsystems.Storage;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Turret;
import frc4388.robot.subsystems.VisionOdometry;

public class AutonomousBuilder {
  private final RobotContainer m_robotContainer;
  private final SwerveDrive m_drive;
  private final Extender m_extender;
  private final Intake m_intake;
  private final Serializer m_serializer;
  private final Storage m_storage;
  private final BoomBoom m_boomBoom;
  private final Turret m_turret;
  private final Hood m_hood;
  private final VisionOdometry m_visionOdometry;

  public AutonomousBuilder(RobotContainer robotContainer) {
    m_robotContainer = robotContainer;
    m_drive = m_robotContainer.m_robotSwerveDrive;
    m_extender = m_robotContainer.m_robotExtender;
    m_intake = m_robotContainer.m_robotIntake;
    m_serializer = m_robotContainer.m_robotSerializer;
    m_storage = m_robotContainer.m_robotStorage;
    m_boomBoom = m_robotContainer.m_robotBoomBoom;
    m_turret = m_robotContainer.m_robotTurret;
    m_hood = m_robotContainer.m_robotHood;
    m_visionOdometry = m_robotContainer.m_robotVisionOdometry;
  }

  public Command buildOneBallCommand() {
    return CommandGroupBase.sequence(
      buildStartupCommandPart(),
      // Shoot Preloaded Ball
      buildTimeoutTrackShotGroup(AutoConstants.LOCK_ON_DURATION, AutoConstants.LOCK_ON_TIME_ALLOWANCE, AutoConstants.STORAGE_TIME_ONE_BALL, "FirstBall"),
      buildStopCommandPart()
    ).withName("OneBall");
  }

  public Command buildTwoBallCommand() {
    return CommandGroupBase.sequence(
      buildStartupCommandPart(),
      // Get Second Ball
      new InstantCommand(() -> m_turret.runShooterRotatePID(-180), m_turret).withName("StartTurningShooter"),
      new PathPlannerCommand("JMove1", AutoConstants.PATH_MAX_VEL, AutoConstants.PATH_MAX_ACCEL, m_drive).withName("JMove1"),
      // Shoot Preloaded and Second Ball
      buildTimeoutTrackShotGroup(AutoConstants.LOCK_ON_DURATION, AutoConstants.LOCK_ON_TIME_ALLOWANCE, AutoConstants.STORAGE_TIME_TWO_BALLS, "FirstSecondBall"),
      buildStopCommandPart()
    ).withName("TwoBall");
  }

  public Command buildThreeBallCommand() {
    return CommandGroupBase.sequence(
      buildStartupCommandPart(),
      // Get Second Ball
      new InstantCommand(() -> m_turret.runShooterRotatePID(-180), m_turret).withName("StartTurningShooter"),
      new PathPlannerCommand("JMove1", AutoConstants.PATH_MAX_VEL, AutoConstants.PATH_MAX_ACCEL, m_drive).withName("JMove1"),
      // Shoot Preloaded and Second Ball
      buildTimeoutTrackShotGroup(AutoConstants.LOCK_ON_DURATION, AutoConstants.LOCK_ON_TIME_ALLOWANCE, AutoConstants.STORAGE_TIME_TWO_BALLS, "FirstSecondBall"),
      // Get Third Ball
      new InstantCommand(() -> m_boomBoom.runDrumShooterVelocityPID(8000), m_boomBoom).withName("StartIdlingShooter"),
      new InstantCommand(() -> m_turret.runShooterRotatePID(-120), m_turret).withName("StartTurningShooter"),
      new PathPlannerCommand("JMove2", AutoConstants.PATH_MAX_VEL, AutoConstants.PATH_MAX_ACCEL, m_drive).withName("JMove2"),
      // Shoot Third Ball
      buildTimeoutTrackShotGroup(AutoConstants.LOCK_ON_DURATION, AutoConstants.LOCK_ON_TIME_ALLOWANCE, AutoConstants.STORAGE_TIME_ONE_BALL, "ThirdBall"),
      buildStopCommandPart()
    ).withName("ThreeBall");
  }

  private Command buildStartupCommandPart() {
    return CommandGroupBase.sequence(
      new InstantCommand(() -> m_boomBoom.runDrumShooterVelocityPID(8000), m_boomBoom).withName("StartIdlingShooter"),
      new InstantCommand(() -> m_intake.runAtOutput(-1), m_intake).withName("StartRunningIntake"),
      new DeployExtender(m_extender, m_intake).withName("DeployExtender")
    );
  }

  private Command buildStopCommandPart() {
    return CommandGroupBase.sequence(
      new InstantCommand(() -> m_intake.runAtOutput(0), m_intake).withName("StopRunningIntake"),
      new InstantCommand(() -> m_serializer.setSerializer(0.0), m_serializer).withName("StopRunningSerializer"),
      new InstantCommand(() -> m_storage.runStorage(0.0)).withName("StopRunningStorage")
    );
  }

  private ParallelDeadlineGroup buildTimeoutTrackShotGroup(double lockOnDuration, double lockOnTimeAllowance, double storageRunTime, String name) {
    return CommandGroupBase.sequence(
      new TimedWaitUntilCommand(m_robotContainer::isLockedOn, lockOnDuration).withTimeout(lockOnTimeAllowance).withName(name + "LockOn"),
      new InstantCommand(() -> m_storage.runStorage(StorageConstants.STORAGE_SPEED), m_storage).withName(name + "Feed"),
      new WaitCommand(storageRunTime).withName(name + "ShootTimer"),
      new InstantCommand(() -> m_storage.runStorage(0.0)).withName(name + "StopFeed")
    ).deadlineWith(new TrackTarget(m_visionOdometry, m_turret, m_hood, m_boomBoom).withName(name + "Track"));
  }
}
