package frc4388.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.Gains;

public class Hood extends SubsystemBase {
  public BoomBoom m_shooterSubsystem;
  
  public CANSparkMax m_angleAdjusterMotor;
  // public SparkMaxLimitSwitch m_hoodUpLimitSwitch;
  // public SparkMaxLimitSwitch m_hoodDownLimitSwitch;
  public static Gains m_angleAdjusterGains;
  public RelativeEncoder m_angleEncoder;

  public SparkMaxPIDController m_angleAdjusterPIDController;
  
  
  public boolean m_isHoodReady = false;

  public double m_fireAngle;
  
  public double speedLimiter;
  public double calibrationSpeed = 1.0;

  /** Creates a new Hood. */
  public Hood(CANSparkMax angleAdjusterMotor) {

    m_angleAdjusterMotor = angleAdjusterMotor;
    m_angleEncoder = m_angleAdjusterMotor.getEncoder();
    m_angleAdjusterPIDController = m_angleAdjusterMotor.getPIDController();
    m_angleAdjusterGains = ShooterConstants.SHOOTER_ANGLE_GAINS;

    // m_hoodUpLimitSwitch = m_angleAdjusterMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    // m_hoodDownLimitSwitch = m_angleAdjusterMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    // m_hoodUpLimitSwitch.enableLimitSwitch(true);
    // m_hoodDownLimitSwitch.enableLimitSwitch(true);

    m_angleAdjusterMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ShooterConstants.HOOD_FORWARD_SOFT_LIMIT);
    m_angleAdjusterMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ShooterConstants.HOOD_REVERSE_SOFT_LIMIT);
    setHoodSoftLimits(true);

    this.speedLimiter = 1.0;
  }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood Angle", m_angleEncoder.getPosition());

    // * speed limiting near soft limits. tolerance (distance when ramping starts) is 20 rotations. speed at hard limits is 0.2 (percent output).
    // runVelocityRamping();
  }

  public void runVelocityRamping() {
    if (areSoftLimitsEnabled()) {
      double currentPos = this.getEncoderPosition();
      double forwardDistance = Math.abs(currentPos - ShooterConstants.HOOD_FORWARD_SOFT_LIMIT);
      double reverseDistance = Math.abs(currentPos - ShooterConstants.HOOD_REVERSE_SOFT_LIMIT);

      if (forwardDistance < ShooterConstants.HOOD_SOFT_LIMIT_TOLERANCE) {
        this.speedLimiter = 0.2 + (forwardDistance * (1 / ShooterConstants.HOOD_SOFT_LIMIT_TOLERANCE));
      }

      if (reverseDistance < ShooterConstants.HOOD_SOFT_LIMIT_TOLERANCE) {
        this.speedLimiter = 0.2 + (reverseDistance * (1 / ShooterConstants.HOOD_SOFT_LIMIT_TOLERANCE));
      }

      if ((forwardDistance > ShooterConstants.HOOD_SOFT_LIMIT_TOLERANCE) && (reverseDistance > ShooterConstants.HOOD_SOFT_LIMIT_TOLERANCE)) {
        this.speedLimiter = 1.0;
      }
    }
  }

  public boolean areSoftLimitsEnabled() {
    return this.m_angleAdjusterMotor.isSoftLimitEnabled(SoftLimitDirection.kForward) && this.m_angleAdjusterMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
  }

    /**
   * Set status of hood motor soft limits.
   * @param set Boolean to set soft limits to.
   */
  public void setHoodSoftLimits(boolean set) {
    m_angleAdjusterMotor.enableSoftLimit(SoftLimitDirection.kForward, set);
    m_angleAdjusterMotor.enableSoftLimit(SoftLimitDirection.kReverse, set);
  }

  public void runAngleAdjustPID(double targetAngle) 
  {
    //Set PID Coefficients
    m_angleAdjusterPIDController.setP(m_angleAdjusterGains.kP);
    m_angleAdjusterPIDController.setI(m_angleAdjusterGains.kI);
    m_angleAdjusterPIDController.setD(m_angleAdjusterGains.kD);
    m_angleAdjusterPIDController.setIZone(m_angleAdjusterGains.kIzone);
    m_angleAdjusterPIDController.setFF(m_angleAdjusterGains.kF);
    m_angleAdjusterPIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN, m_angleAdjusterGains.kPeakOutput);

    m_angleAdjusterPIDController.setReference(targetAngle, ControlType.kPosition);
  }
  
 /**
  * Runs the hood with the given input
  * @param input value from -1.0 to 1.0, postive is upward (more horizontal shootijng angle)
  */
  public void runHood(double input) {
    m_angleAdjusterMotor.set(input * this.speedLimiter * this.calibrationSpeed);
  }

  /**
   * Run a PID to go to the zero position.
   */
  public void gotoZero() {
    runAngleAdjustPID(0);
  }

  public void resetGyroAngleAdj() {
    m_angleEncoder.setPosition(0);
  }

  public double getEncoderPosition(){
    return m_angleEncoder.getPosition();
  }

  public double getAnglePositionDegrees(){
    return 0.0;//((m_angleEncoder.getPosition() - ShooterConstants.HOOD_MOTOR_POS_AT_ZERO_ROT) * 360/ShooterConstants.HOOD_MOTOR_ROTS_PER_ROT) - 90;
  }

  public double getCurrent(){
    return m_angleAdjusterMotor.getOutputCurrent();
  }

  public boolean isLockedOn() {
    double currentHood = getEncoderPosition();
    double targetHood = m_angleAdjusterMotor.get();
    return Math.abs(currentHood - targetHood) > HOOD_TOLERANCE;
  }
  private static final double HOOD_TOLERANCE = 5.0;
}
