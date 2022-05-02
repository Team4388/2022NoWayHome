package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClimberConstants;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class Climber extends SubsystemBase {
  private WPI_TalonFX elbow;

  /** Creates a new Climber */
  public Climber(WPI_TalonFX elbow) {
    this.elbow = elbow;
    elbow.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    elbow.overrideLimitSwitchesEnable(true); //Check
    
    elbow.configReverseSoftLimitThreshold(ClimberConstants.ELBOW_FORWARD_SOFT_LIMIT); // Tune
    elbow.configReverseSoftLimitEnable(false);
  }

  public void setEncoders(double value) {
    this.elbow.setSelectedSensorPosition(value);
  }

  public double getCurrent() {
    return this.elbow.getSupplyCurrent();
  }

  public void setMotors(double elbowOutput) {
    this.elbow.set(elbowOutput * ClimberConstants.INPUT_MULTIPLIER);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elbow Angle", elbow.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Climber Folded", (elbow.isFwdLimitSwitchClosed() == 1));
  }
}