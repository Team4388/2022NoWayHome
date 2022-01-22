package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.HooksConstants;

public class Hooks extends SubsystemBase {
  private Servo m_leftHook;
  private Servo m_rightHook;

  private boolean m_open;

  public Hooks(Servo leftHook, Servo rightHook) {
    m_leftHook = leftHook;
    m_rightHook = rightHook;

    m_open = false;
    setOpen(m_open);
  }

  public void setOpen(boolean open) {
    if(open) {
      m_leftHook.setPosition(HooksConstants.OPEN_POSITION);
      m_rightHook.setPosition(HooksConstants.OPEN_POSITION);
    } else {
      m_leftHook.setPosition(HooksConstants.CLOSE_POSITION);
      m_rightHook.setPosition(HooksConstants.CLOSE_POSITION);
    }
  }

  public boolean getOpen() {
    return m_open;
  }
}
