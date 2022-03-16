package frc4388.robot.subsystems;

import java.nio.file.ClosedWatchServiceException;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClawConstants;
import frc4388.robot.Constants.ClimberConstants;

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
  public static enum ClawType {LEFT, RIGHT}

  public Claws(/*CANSparkMax*/Servo leftClaw, /*CANSparkMax*/Servo rightClaw) {
    m_leftClaw = leftClaw;
    m_rightClaw = rightClaw;

    // m_leftLimitSwitchF = m_leftClaw.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    // m_rightLimitSwitchF = m_rightClaw.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    // m_leftLimitSwitchR = m_leftClaw.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed); //Wired wrong lol
    // m_rightLimitSwitchR = m_rightClaw.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // m_leftLimitSwitchF.enableLimitSwitch(true);
    // m_rightLimitSwitchF.enableLimitSwitch(true);
    // m_leftLimitSwitchR.enableLimitSwitch(true);
    // m_rightLimitSwitchR.enableLimitSwitch(true);

    // m_leftClaw.setInverted(true);
    // m_rightClaw.setInverted(true);

    m_open = false;

    // m_leftClaw.set(ClawConstants.CALIBRATION_SPEED);
    // m_rightClaw.set(ClawConstants.CALIBRATION_SPEED);
  }

   /**
   * Run a specific claw to open or close.
   * @param which Which claw to run.
   * @param open Whether to open or close the claw.
   */
  public void runClaw(ClawType which, boolean open) {
    
    int direction = open ? 1 : -1;

    if (which == Claws.ClawType.LEFT) {

      // double setPos = open ? ClawsConstants.OPEN_POSITION + m_leftOffset : ClawsConstants.CLOSE_POSITION + m_leftOffset;
      // m_leftClaw.getEncoder().setPosition(setPos);
      m_leftClaw.setSpeed(direction * 0.1);

    } else if (which == Claws.ClawType.RIGHT) {

      // double setPos = open ? ClawsConstants.OPEN_POSITION + m_rightOffset : ClawsConstants.CLOSE_POSITION + m_rightOffset;
      // m_rightClaw.getEncoder().setPosition(setPos);
      m_rightClaw.setSpeed(direction * 0.1);
    }

    m_open = open;
  }

  public void runClaw(ClawType which, double input) {
    if (which == Claws.ClawType.LEFT) {
      m_leftClaw.setSpeed(input);

    } else if (which == Claws.ClawType.RIGHT) {
      m_rightClaw.setSpeed(input);
    }
  }

  public void runClaws(double input)
  {
    m_leftClaw.setSpeed(input);
    m_rightClaw.setSpeed(input);
  }

  /**
   * Sets the state of both hooks
   * @param open The state of the hooks
   */
  public void setOpen(boolean open) {
    if(open) {
      // m_leftClaw.getEncoder().setPosition(ClawsConstants.OPEN_POSITION + m_leftOffset);
      // m_rightClaw.getEncoder().setPosition(ClawsConstants.OPEN_POSITION + m_rightOffset);
      m_leftClaw.set(0.1);
      m_rightClaw.set(0.1);
    } else {
      // m_leftClaw.getEncoder().setPosition(ClawsConstants.CLOSE_POSITION + m_leftOffset);
      // m_rightClaw.getEncoder().setPosition(ClawsConstants.CLOSE_POSITION + m_rightOffset);
      m_leftClaw.set(-0.1);
      m_rightClaw.set(-0.1);
    }
  }

  public double[] getOffsets() {
    return new double[] {m_leftOffset, m_rightOffset};
  }

  public boolean fullyOpen() {
    return Math.abs(m_leftClaw.getPosition() - ClawConstants.OPEN_POSITION) < ClawConstants.THRESHOLD &&
        Math.abs(m_rightClaw.getPosition() - ClawConstants.OPEN_POSITION) < ClawConstants.THRESHOLD;  }

  public boolean fullyClosed() {
    return Math.abs(m_leftClaw.getPosition() - ClawConstants.CLOSE_POSITION) < ClawConstants.THRESHOLD &&
        Math.abs(m_rightClaw.getPosition() - ClawConstants.CLOSE_POSITION) < ClawConstants.THRESHOLD;
  }

  /**
   * Check if a limit switch is pressed or current limit exceeded for a claw.
   * @param which Which claw to check.
   * @param limit The current limit.
   * @return Whether to interrupt the RunClaw command or not.
   */
  // public boolean checkSwitchAndCurrent(ClawType which) {    
  //   if (which == ClawType.LEFT) {
  //     if (m_leftLimitSwitchF.isPressed() || m_leftLimitSwitchR.isPressed() || m_leftClaw.getOutputCurrent() >= ClawConstants.CURRENT_LIMIT) {
  //       return true;
  //     }
  //   } 
  //   else if (which == ClawType.RIGHT) {
  //     if (m_rightLimitSwitchF.isPressed() || m_rightLimitSwitchR.isPressed() || m_rightClaw.getOutputCurrent() >= ClawConstants.CURRENT_LIMIT) {
  //       return true;
  //     }
  //   }
  //   return false;
  // }

  @Override
  public void periodic() {
    if (fullyOpen() || fullyClosed()) {
      m_leftClaw.setSpeed(0.0);
      m_rightClaw.setSpeed(0.0);
    }

  //   if(m_leftLimitSwitchF.isPressed() || m_leftLimitSwitchR.isPressed())
  //     // m_leftOffset = m_leftClaw.getEncoder().getPosition();
  //     m_leftOffset = m_leftClaw.getPosition();
    
  //   if(m_rightLimitSwitchF.isPressed() || m_rightLimitSwitchR.isPressed())
  //     // m_rightOffset = m_rightClaw.getEncoder().getPosition();
  //     m_rightOffset = m_rightClaw.getPosition();
  }
}