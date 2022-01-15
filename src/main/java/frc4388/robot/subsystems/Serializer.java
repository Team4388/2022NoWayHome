package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;

public class Serializer extends SubsystemBase{
  private Spark m_serializerBelt;
  private Spark m_serializerShooterBelt;

  private boolean serializerState;

    public Serializer(Spark serializerBelt, Spark serializerShooterBelt) {
        m_serializerBelt = serializerBelt;
        m_serializerShooterBelt = serializerShooterBelt;

        serializerState = false;
        setSerializerState(serializerState);
    }

    public void setSerializerState(boolean state) {
      double serializerBeltSpeed = state ? Constants.SerializerConstants.SERIALIZER_BELT_SPEED : 0.d;
      double serializerShooterBeltSpeed = state ? Constants.SerializerConstants.SERIALIZER_SHOOTER_BELT_SPEED : 0.d;

      m_serializerBelt.set(serializerBeltSpeed);
      m_serializerShooterBelt.set(serializerShooterBeltSpeed);

      serializerState = state;
    }
}
