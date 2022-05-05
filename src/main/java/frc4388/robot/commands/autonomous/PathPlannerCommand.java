package frc4388.robot.commands.autonomous;

import java.util.function.Function;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.SwerveDrive;

public class PathPlannerCommand extends SequentialCommandGroup {
  private static final Logger LOGGER = Logger.getLogger(PathPlannerCommand.class.getSimpleName());
  /** Function that removes the ".path" from the end of a string. */
  private static final Function<CharSequence, String> PATH_EXTENSION_REMOVER = ((Function<CharSequence, Matcher>) Pattern.compile(".path")::matcher).andThen(m -> m.replaceFirst(""));
  private final SwerveDrive m_swerveDrive;
  public PathPlannerCommand(String pathFileName, SwerveDrive swerveDrive) {
    this(pathFileName, SwerveDriveConstants.PATH_MAX_VELOCITY, SwerveDriveConstants.PATH_MAX_ACCELERATION, swerveDrive);
  }
  public PathPlannerCommand(String pathName, double maxVel, double maxAccel, SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
    String pathFileName = PATH_EXTENSION_REMOVER.apply(pathName);
    setName(pathFileName);

    PIDController xController = SwerveDriveConstants.X_CONTROLLER;
    PIDController yController = SwerveDriveConstants.Y_CONTROLLER;
    ProfiledPIDController thetaController = SwerveDriveConstants.THETA_CONTROLLER;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    LOGGER.log(Level.WARNING, "Loading path {0}.", pathFileName);
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathFileName, maxVel, maxAccel);
    if (pathTrajectory != null) {
      LOGGER.info("Done loading.");
      m_swerveDrive.m_field.getObject("traj").setTrajectory(pathTrajectory);
      PathPlannerState initialState = pathTrajectory.getInitialState();
      Pose2d initialPosition = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

      addCommands(
        new InstantCommand(() -> m_swerveDrive.resetOdometry(initialPosition), m_swerveDrive),
        new PPSwerveControllerCommand(pathTrajectory, m_swerveDrive::getPoseMeters, SwerveDriveConstants.DRIVE_KINEMATICS, xController, yController, thetaController, m_swerveDrive::setModuleStates, m_swerveDrive),
        new InstantCommand(m_swerveDrive::stopModules, m_swerveDrive)
      );
    } else {
      LOGGER.log(Level.SEVERE, "Failed loading path {0}.", pathFileName);
      addCommands(new PrintCommand("Path failed to load."));
    }
  }
}
