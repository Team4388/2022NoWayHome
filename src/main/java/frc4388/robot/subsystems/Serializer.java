package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SerializerConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;

public class Serializer extends SubsystemBase{
  private CANSparkMax m_serializerBelt;
  // private CANSparkMax m_serializerShooterBelt;
  private DigitalInput m_serializerBeam;
  private boolean serializerState;

  public Serializer(CANSparkMax serializerBelt, /*CANSparkMax serializerShooterBelt,*/ DigitalInput serializerBeam) { 
    m_serializerBelt = serializerBelt;
    m_serializerBeam = serializerBeam;

    m_serializerBelt.set(0);
    // m_serializerShooterBelt.set(0);
    
  }

  public void setSerializer(double input){
    m_serializerBelt.set(input);
  }
  /**
   * Gets The State Of The Beam
   * @return The State Of The Beam
   */
  public boolean getBeam() {
    return m_serializerBeam.get();
  }

  /**
   * Sets The Serializer State With The Beam
   * @param state Your State Of The Button
   * @param beambroken The State of the Beam Senser
   */
  public void setSerializerStateWithBeam() {
    if (m_serializerBeam.get()) setSerializer(0.0);
    else setSerializer(SerializerConstants.SERIALIZER_BELT_SPEED);
  }
  
  /**
   * Gets the Serializer State
   * @return The Serializer State
   */
  public boolean getSerializerState() {
    return serializerState;
  }
}
