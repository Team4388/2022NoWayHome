package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClawConstants;

public class Claws extends SubsystemBase {
  private CANSparkMax m_leftClaw;
  private CANSparkMax m_rightClaw;

  private SparkMaxLimitSwitch m_leftLimitSwitchF;
  private SparkMaxLimitSwitch m_rightLimitSwitchF;
  private SparkMaxLimitSwitch m_leftLimitSwitchR;
  private SparkMaxLimitSwitch m_rightLimitSwitchR;

  private double m_leftOffset;
  private double m_rightOffset;

  private boolean m_open;

  public Claws(CANSparkMax leftClaw, CANSparkMax rightClaw) {
    m_leftClaw = leftClaw;
    m_rightClaw = rightClaw;

    m_leftLimitSwitchF = m_leftClaw.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_rightLimitSwitchF = m_rightClaw.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_leftLimitSwitchR = m_leftClaw.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed); //Wired wrong lol
    m_rightLimitSwitchR = m_rightClaw.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    m_leftLimitSwitchF.enableLimitSwitch(true);
    m_rightLimitSwitchF.enableLimitSwitch(true);
    m_leftLimitSwitchR.enableLimitSwitch(true);
    m_rightLimitSwitchR.enableLimitSwitch(true);
    leftClaw.setInverted(true);
    rightClaw.setInverted(true);

    m_open = false;

    // m_leftClaw.set(ClawConstants.CALIBRATION_SPEED);
    // m_rightClaw.set(ClawConstants.CALIBRATION_SPEED);
  }

  public void setSpeed(double speed){
    m_leftClaw.set(speed);
    m_rightClaw.set(speed);
  }

  public void setOpen(boolean open) {
    if(open) {
      m_leftClaw.getEncoder().setPosition(ClawConstants.OPEN_POSITION + m_leftOffset);
      m_rightClaw.getEncoder().setPosition(ClawConstants.OPEN_POSITION + m_rightOffset);
    } else {
      m_leftClaw.getEncoder().setPosition(ClawConstants.CLOSE_POSITION + m_leftOffset);
      m_rightClaw.getEncoder().setPosition(ClawConstants.CLOSE_POSITION + m_rightOffset);
    }

    m_open = open;
  }

  public boolean getOpen() {
    return m_open;
  }

  @Override
  public void periodic() {
    if(m_leftLimitSwitchR.isPressed())
      m_leftOffset = m_leftClaw.getEncoder().getPosition();
    
    if(m_rightLimitSwitchR.isPressed())
      m_rightOffset = m_rightClaw.getEncoder().getPosition();
  }
}
