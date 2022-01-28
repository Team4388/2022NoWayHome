package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.HooksConstants;

public class Hooks extends SubsystemBase {
  private CANSparkMax m_leftHook;
  private CANSparkMax m_rightHook;

  private LimitSwitchNormal m_limitSwitch;

  private double m_leftOffset;
  private double m_rightOffset;

  private boolean m_open;

  public Hooks(CANSparkMax leftHook, CANSparkMax rightHook, LimitSwitchNormal limitSwitch) {
    m_leftHook = leftHook;
    m_rightHook = rightHook;

    m_limitSwitch = limitSwitch;

    m_open = false;

    m_leftHook.set(.1);
    m_rightHook.set(.1);
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
    if(m_limitSwitch.compareTo(LimitSwitchNormal.NormallyClosed) == 1) {
      m_leftOffset = m_leftHook.getEncoder().getPosition();
      m_leftOffset = m_leftHook.getEncoder().getPosition();
    }
  }
}
