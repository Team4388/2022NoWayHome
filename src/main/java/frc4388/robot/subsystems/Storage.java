package frc4388.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Storage extends SubsystemBase {
  public CANSparkMax m_storageMotor;
  public ColorSensorV3 m_colorSensor;
  

  /** Creates a new Storage. */
  public Storage(CANSparkMax storageMotor, ColorSensorV3 colorSensor) {
    m_storageMotor = storageMotor;
    m_colorSensor = colorSensor;
  }

  public Storage(CANSparkMax storageMotor) {
    m_storageMotor = storageMotor;
    m_colorSensor = null;
  }
  /**
   * If The Beam Is Broken, Run Storage
   * If Else, Stop Running Storage
   */
  public void manageStorage() {
    if (getColorBroken()) runStorage(0.d);
    else runStorage(0.9);
  }
  
  /**
   * Runs The Storage at a Specifyed Speed
   * @param input The value frm -1.0 to 1.0, positive is inwards (towards the shooter)
   */
  public void runStorage(double input) {
    m_storageMotor.set(input);
  }
  /**
   * Gets the state of the colorsensor as a beam break
   * @return The State Of The Beam on the Shooter
   */
  public boolean getColorBroken(){
    return (getRed() || getBlue());
  }

  public boolean getRed(){
    // return (m_colorSensor.getRed() >= 200 && m_colorSensor.getBlue() < 100  && m_colorSensor.getGreen() < 100);
    return (m_colorSensor.getColor() == Color.kRed);
  }

  public boolean getBlue(){
    // return (m_colorSensor.getBlue() >= 200  && m_colorSensor.getRed() < 100  && m_colorSensor.getGreen() < 100);
    return (m_colorSensor.getColor() == Color.kBlue);
  }

  public Alliance getColor() {
    return (getRed() ? Alliance.Red : Alliance.Blue);
  }  

  @Override
  /**
   * Every Robot Tick Manage The Storage
   */
  public void periodic() {
    //manageStorage();
  }
  public double getCurrent(){
    return m_storageMotor.getOutputCurrent();
  }
}
