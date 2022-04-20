package frc4388.robot.commands.extender;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Extender;
import frc4388.robot.subsystems.Intake;

public class DeployExtender extends CommandBase {
  private final Extender m_extender;
  private final Intake m_intake;

  public DeployExtender(Extender extender, Intake intake) {
    m_extender = extender;
    m_intake = intake;
    addRequirements(extender, intake);
  }

  @Override
  public void initialize() {
    m_intake.runAtOutput(-1.0);
    m_extender.runExtender(1.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.runAtOutput(0.0);
    m_extender.runExtender(0.0);
  }

  @Override
  public boolean isFinished() {
    return RobotBase.isSimulation() || m_extender.isDeployed();
  }
}
