package frc4388.robot.commands.extender;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Extender;

public class RetractExtender extends CommandBase {
  private final Extender m_extender;

  public RetractExtender(Extender extender) {
    m_extender = extender;
    addRequirements(extender);
  }

  @Override
  public void initialize() {
    m_extender.runExtender(-1.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_extender.runExtender(0.0);
  }

  @Override
  public boolean isFinished() {
    return RobotBase.isSimulation() || m_extender.isRetracted();
  }
}
