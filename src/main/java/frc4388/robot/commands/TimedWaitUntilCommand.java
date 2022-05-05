package frc4388.robot.commands;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TimedWaitUntilCommand extends CommandBase {
  private final BooleanSupplier m_condition;
  private final double m_duration;

  protected Timer m_timer = new Timer();

  public TimedWaitUntilCommand(BooleanSupplier condition, double seconds, Subsystem... requirements) {
    m_condition = requireNonNullParam(condition, "condition", "TimedWaitUntilCommand");
    m_duration = seconds;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public boolean isFinished() {
    if (m_condition.getAsBoolean())
      return m_timer.hasElapsed(m_duration);
    m_timer.reset();
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }
}
