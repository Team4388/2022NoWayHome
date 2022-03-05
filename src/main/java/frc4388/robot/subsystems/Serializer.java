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

  public Serializer(CANSparkMax serializerBelt, CANSparkMax serializerShooterBelt, DigitalInput serializerBeam) { //TODO: Only one motor lol
    m_serializerBelt = serializerBelt;
    m_serializerShooterBelt = serializerShooterBelt;
    m_serializerBeam = serializerBeam;

    serializerState = false;
    setSerializerState(serializerState);
    m_serializerBelt.set(0);
    m_serializerShooterBelt.set(0);
    
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
  public void setSerializerStateWithBeam(boolean state, boolean beambroken) {
    boolean total = state || beambroken;
    setSerializerState(total);
  }
  /**
   * Sets The Serializer State With The Beam 
   * @param state Your State Of The Button
   */
  public void setSerializerState(boolean state) {
    setSerializerBeltState(state);
    setSerializerShooterBeltState(state);
    serializerState = state;
  }
  /**
   * Sets the Serializer Belt State
   * @param state Your State Of The Button
   */
  public void setSerializerBeltState(boolean state) {
    double serializerBeltSpeed = state ? Constants.SerializerConstants.SERIALIZER_BELT_SPEED : 0.d;
    m_serializerBelt.set(serializerBeltSpeed);
  }
  /**
   * Sets the Shooter Belt State
   * @param state Your State Of The Button
   */
  public void setSerializerShooterBeltState(boolean state) {
    double serializerShooterBeltSpeed = state ? Constants.SerializerConstants.SERIALIZER_SHOOTER_BELT_SPEED : 0.d;
    m_serializerShooterBelt.set(serializerShooterBeltSpeed);
  }
  
  /**
   * Gets the Serializer State
   * @return The Serializer State
   */
  public boolean getSerializerState() {
    return serializerState;
  }
}
