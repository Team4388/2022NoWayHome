package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

public class Serializer extends SubsystemBase{
  private CANSparkMax m_serializerBelt;
  private boolean serializerState;

  public Serializer(CANSparkMax serializerBelt) { 
    m_serializerBelt = serializerBelt;

    m_serializerBelt.set(0);
    
  }
  /**
   * 
   * @param input from -1.0 to 1.0, positive is inward
   */
  public void setSerializer(double input){
    m_serializerBelt.set(input);
  }
  
  /**
   * Gets the Serializer State
   * @return The Serializer State
   */
  public boolean getSerializerState() {
    return serializerState;
  }

  public double getCurrent(){
    return m_serializerBelt.getOutputCurrent();
  }
}
