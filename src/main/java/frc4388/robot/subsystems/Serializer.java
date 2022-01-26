package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;

public class Serializer extends SubsystemBase{
  private CANSparkMax m_serializerBelt;
  private CANSparkMax m_serializerShooterBelt;
  private DigitalInput m_serializerBeam;
  private boolean serializerState;

  public Serializer(CANSparkMax serializerBelt, CANSparkMax serializerShooterBelt) {
    m_serializerBelt = serializerBelt;
    m_serializerShooterBelt = serializerShooterBelt;
    //m_serializerBeam = new DigitalInput(Constants.SerializerConstants.SERIALIZER_BELT_BEAM);

    serializerState = false;
    setSerializerState(serializerState);
    m_serializerBelt.set(0);
    m_serializerShooterBelt.set(0);
    
  }
  public boolean getBeam() {
    System.out.println("oi");
    return false;
  }
  public void setSerializerStateWithBeam(boolean ctrlbutter, boolean beambroken) {
    boolean total = ctrlbutter || beambroken;
    setSerializerState(total);
  }
  public void setSerializerState(boolean state) {
    setSerializerBeltState(state);
    setSerializerShooterBeltState(state);
    serializerState = state;
  }

  public void setSerializerBeltState(boolean state) {
    double serializerBeltSpeed = state ? Constants.SerializerConstants.SERIALIZER_BELT_SPEED : 0.d;
    m_serializerBelt.set(serializerBeltSpeed);
  }

  public void setSerializerShooterBeltState(boolean state) {
    double serializerShooterBeltSpeed = state ? Constants.SerializerConstants.SERIALIZER_SHOOTER_BELT_SPEED : 0.d;
    m_serializerShooterBelt.set(serializerShooterBeltSpeed);
  }
  
  public boolean getSerializerState() {
    return serializerState;
  }
}
