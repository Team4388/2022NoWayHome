package frc4388.utility.shuffleboard;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableCANSparkMax extends CANSparkMax implements Sendable {
  public SendableCANSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Motor Controller");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Value", this::get, this::set);
  }

}
