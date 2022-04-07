package frc4388.utility.shuffleboard;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableCANSparkMax implements Sendable {
  private final CANSparkMax m_canSparkMax;

  public SendableCANSparkMax(CANSparkMax canSparkMax) {
    m_canSparkMax = canSparkMax;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Motor Controller");
    builder.setActuator(true);
    builder.setSafeState(m_canSparkMax::stopMotor);
    builder.addDoubleProperty("Value", m_canSparkMax::get, m_canSparkMax::set);
  }

}
