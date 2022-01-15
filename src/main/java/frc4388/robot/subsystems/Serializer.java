package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Serializer extends SubsystemBase{
  private Spark m_serializerBelt;
  private Spark m_serializerShooterBelt;
  private DigitalInput m_serializerBeam;
  private boolean serializerState;

  public Serializer(Spark serializerBelt, Spark serializerShooterBelt) {
    m_serializerBelt = serializerBelt;
    m_serializerShooterBelt = serializerShooterBelt;
    m_serializerBeam = new DigitalInput(Constants.SerializerConstants.SERIALIZER_BELT_BEAM);

    serializerState = false;
    setSerializerState(serializerState);
  }
  public boolean getBeam() {
    return m_serializerBeam.get();
  }
  public void setSerializerStateWithBeam(boolean ctrlbutter, boolean beambroken) {
    boolean total = ctrlbutter || beambroken;
    setSerializerState(total);
  }
  public void setSerializerState(boolean state) {
    double serializerBeltSpeed = state ? Constants.SerializerConstants.SERIALIZER_BELT_SPEED : 0.d;
    double serializerShooterBeltSpeed = state ? Constants.SerializerConstants.SERIALIZER_SHOOTER_BELT_SPEED : 0.d;

    m_serializerBelt.set(serializerBeltSpeed);
    m_serializerShooterBelt.set(serializerShooterBeltSpeed);

    serializerState = state;
  }
  
  public boolean getSerializerState() {
    return serializerState;
  }
}
