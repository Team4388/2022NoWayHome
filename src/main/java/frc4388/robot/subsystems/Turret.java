// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;


import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.Gains;
import com.revrobotics.CANSparkMax.SoftLimitDirection;




public class Turret extends SubsystemBase {
 
  /** Creates a new Turret. */
  public BoomBoom m_boomBoomSubsystem;
  public SwerveDrive m_sDriveSubsystem;

  public CANSparkMax m_boomBoomRotateMotor;// = new CANSparkMax(ShooterConstants.SHOOTER_ROTATE_ID, MotorType.kBrushless);
  public static Gains m_shooterTGains = ShooterConstants.SHOOTER_TURRET_GAINS;
  SparkMaxLimitSwitch m_boomBoomRightLimit, m_boomBoomLeftLimit;
  public Gyro m_turretGyro;

  public double m_targetDistance = 0;

  public boolean m_isAimReady = false;

  SparkMaxPIDController m_boomBoomRotatePIDController;// = m_boomBoomRotateMotor.getPIDController();
  public RelativeEncoder m_boomBoomRotateEncoder;// = m_boomBoomRotateMotor.getEncoder();
  
  //Variables
  public Turret(CANSparkMax boomBoomRotateMotor) { //Take in rotate motor as an argument

    m_boomBoomRotateMotor = boomBoomRotateMotor;
    m_boomBoomRotatePIDController = m_boomBoomRotateMotor.getPIDController();
    m_boomBoomRotateEncoder = m_boomBoomRotateMotor.getEncoder();
    m_boomBoomRotateMotor.setIdleMode(IdleMode.kBrake);
  
  
    m_boomBoomLeftLimit = m_boomBoomRotateMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_boomBoomRightLimit = m_boomBoomRotateMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_boomBoomRightLimit.enableLimitSwitch(true);
    m_boomBoomLeftLimit.enableLimitSwitch(true);

    m_boomBoomRotateMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_boomBoomRotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_boomBoomRotateMotor.setSoftLimit(SoftLimitDirection.kForward, ShooterConstants.TURRET_RIGHT_SOFT_LIMIT); //Set second soft limit
    m_boomBoomRotateMotor.setInverted(false);

    m_boomBoomRotatePIDController.setP(m_shooterTGains.m_kP);
    m_boomBoomRotatePIDController.setI(m_shooterTGains.m_kI);
    m_boomBoomRotatePIDController.setD(m_shooterTGains.m_kD);
    m_boomBoomRotatePIDController.setFF(m_shooterTGains.m_kF);
    m_boomBoomRotatePIDController.setIZone(m_shooterTGains.m_kIzone);
    m_boomBoomRotatePIDController.setOutputRange(ShooterConstants.SHOOTER_TURRET_MIN, m_shooterTGains.m_kPeakOutput);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void passRequiredSubsystem( BoomBoom subsystem0, SwerveDrive subsystem1){
    m_boomBoomSubsystem = subsystem0;
    m_sDriveSubsystem = subsystem1;
  }

  public void runTurretWithInput(double input) {
    m_boomBoomRotateMotor.set(input*ShooterConstants.TURRET_SPEED_MULTIPLIER);
  }

  public void runshooterRotatePID(double targetAngle) {
    targetAngle = targetAngle/ShooterConstants.DEGREES_PER_ROT;
    m_boomBoomRotatePIDController.setReference(targetAngle,ControlType.kPosition);
  }

  public void resetGyroShooterRotate()
  {
    m_boomBoomRotateEncoder.setPosition(0);
  }

  public double getboomBoomRotatePosition()
  {
    return m_boomBoomRotateEncoder.getPosition();
  }

  public double getBoomBoomAngleDegrees() {
    return (m_boomBoomRotateEncoder.getPosition() - ShooterConstants.TURRET_MOTOR_POS_AT_ZERO_ROT) * 360/ShooterConstants. TURRET_MOTOR_ROTS_PER_ROT;
  }

}