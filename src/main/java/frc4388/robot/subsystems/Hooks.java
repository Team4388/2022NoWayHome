package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.HooksConstants;

public class Hooks extends SubsystemBase {
  private CANSparkMax m_leftHook;
  private CANSparkMax m_rightHook;

  private DigitalInput m_leftLimitSwitch;
  private DigitalInput m_rightLimitSwitch;

  private double m_leftOffset;
  private double m_rightOffset;

  private boolean m_open;

  public Hooks(CANSparkMax leftHook, CANSparkMax rightHook, DigitalInput leftLimitSwitch, DigitalInput rightLimitSwitch) {
    m_leftHook = leftHook;
    m_rightHook = rightHook;

    m_leftLimitSwitch = leftLimitSwitch;
    m_rightLimitSwitch = rightLimitSwitch;

    m_open = false;

    m_leftHook.set(HooksConstants.CALIBRATION_SPEED);
    m_rightHook.set(HooksConstants.CALIBRATION_SPEED);
  }

  public void setOpen(boolean open) {
    if(open) {
      m_leftHook.getEncoder().setPosition(HooksConstants.OPEN_POSITION + m_leftOffset);
      m_rightHook.getEncoder().setPosition(HooksConstants.OPEN_POSITION + m_rightOffset);
    } else {
      m_leftHook.getEncoder().setPosition(HooksConstants.CLOSE_POSITION + m_leftOffset);
      m_rightHook.getEncoder().setPosition(HooksConstants.CLOSE_POSITION + m_rightOffset);
    }

    m_open = open;
  }

  public boolean getOpen() {
    return m_open;
  }

  @Override
  public void periodic() {
    if(m_leftLimitSwitch.get())
      m_leftOffset = m_leftHook.getEncoder().getPosition();
    
    if(m_rightLimitSwitch.get())
      m_rightOffset = m_rightHook.getEncoder().getPosition();
  }
}
