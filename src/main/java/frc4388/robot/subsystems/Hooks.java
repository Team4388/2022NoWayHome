package frc4388.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.HooksConstants;

public class Hooks extends SubsystemBase {
  private CANSparkMax m_leftHook;
  private CANSparkMax m_rightHook;

  double m_leftOffset;
  double m_rightOffset;

  private boolean m_open;

  public Hooks(CANSparkMax leftHook, CANSparkMax rightHook) {
    m_leftHook = leftHook;
    m_rightHook = rightHook;

    setOpen(false);
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
}
