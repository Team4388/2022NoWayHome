package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClawConstants;

public class Claws extends SubsystemBase {

  public Servo m_leftClaw;
  public Servo m_rightClaw;

  // public CANSparkMax m_leftClaw;
  // public CANSparkMax m_rightClaw;

  // private SparkMaxLimitSwitch m_leftLimitSwitchF;
  // private SparkMaxLimitSwitch m_rightLimitSwitchF;
  // private SparkMaxLimitSwitch m_leftLimitSwitchR;
  // private SparkMaxLimitSwitch m_rightLimitSwitchR;

  private double m_leftOffset;
  private double m_rightOffset;

  private boolean m_open;
  private boolean clawsOpen;
  public static enum ClawType {LEFT, RIGHT}

  public Claws(Servo leftClaw, Servo rightClaw) {
    m_leftClaw = leftClaw;
    m_rightClaw = rightClaw;
    m_open = false;
    clawsOpen = false;
  }
  /**
   * Sets the state of both hooks
   * @param open The state of the hooks
   */
  public void setOpen(boolean open) {
    if(open) {
      m_leftClaw.setRaw(ClawConstants.BOTTOM_LIMIT - 900);
      m_rightClaw.setRaw(ClawConstants.TOP_LIMIT + 100);
      clawsOpen = false;
    } else {
      m_leftClaw.setRaw(ClawConstants.TOP_LIMIT - 400);
      m_rightClaw.setRaw(ClawConstants.BOTTOM_LIMIT - 400);
      clawsOpen = true;
    }
  }

  public void toggleClaws() {
    m_open = !m_open;
    setOpen(m_open);
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Claws Closed", clawsOpen);
  }
}