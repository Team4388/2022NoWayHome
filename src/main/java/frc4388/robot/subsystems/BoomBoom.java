// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Comparator;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Pattern;
import java.util.stream.IntStream;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.diffplug.common.base.Errors;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.CSV;
import frc4388.utility.Gains;
import frc4388.utility.NumericData;

public class BoomBoom extends SubsystemBase {
  private static final Logger LOGGER = Logger.getLogger(BoomBoom.class.getSimpleName());
  public WPI_TalonFX m_shooterFalconLeft;
  public WPI_TalonFX m_shooterFalconRight;
  public static Gains m_drumShooterGains = ShooterConstants.DRUM_SHOOTER_GAINS;
  public static BoomBoom m_boomBoom;
  double speed2;

  double velP;
  double input;
  public double pidOffset = 0;

  public boolean m_isDrumReady = false;
  public double m_fireVel;

  public Hood m_hoodSubsystem;
  public Turret m_turretSubsystem;

  // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(69, 42, 0); //get real values later

  public static class ShooterTableEntry {
    public Double distance, hoodExt, drumVelocity;
  }

  private ShooterTableEntry[] m_shooterTable;

  /** Creates a new BoomBoom, which has a drum shooter and angle adjuster. */
  public BoomBoom(WPI_TalonFX shooterFalconLeft, WPI_TalonFX shooterFalconRight) {
    m_shooterFalconLeft = shooterFalconLeft;
    m_shooterFalconRight = shooterFalconRight;

    setShooterGains();

    m_shooterTable = readShooterTable();
    // Run a helper method that logs the contents of the table on a new thread.
    new Thread(() -> LOGGER.fine(() -> CSV.ReflectionTable.create(m_shooterTable, RobotBase.isSimulation()))).start();
  }

  /**
   * This is a function that takes a value (distance) and returns a value (drumVelocity) that is a
   * linear interpolation of the two values (drumVelocity) at the two closest points in the table
   * (m_shooterTable) to the given value (distance). 
   * @param distance Distance in shooter table
   * @return Drum Velocity in units per 100 ms
   */
  public Double getVelocity(final Double distance) {
    return NumericData.linearInterpolate(m_shooterTable, distance, e -> e.distance, e -> e.drumVelocity).doubleValue();
  }

  /**
   * This is a function that takes a value (distance) and returns a value (hoodExt) that is a linear
   * interpolation of the two values (hoodExt) at the two closest points in the table (m_shooterTable)
   * to the given value (distance).
   * @param distance Distance in shooter table
   * @return Hood extension in units
   */
  public Double getHood(final Double distance) {
    return NumericData.linearInterpolate(m_shooterTable, distance, e -> e.distance, e -> e.hoodExt).doubleValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // speed2 = SmartDashboard.getNumber("Shooter Offset", 0.0);
    // SmartDashboard.putNumber("Shooter Current", getCurrent());
    // SmartDashboard.putNumber("Shooter Voltage",  m_shooterFalconLeft.getMotorOutputVoltage());
    SmartDashboard.putNumber("Shooter RPM", (m_shooterFalconLeft.getSelectedSensorVelocity() * 10) / 2048);
  }

  public void passRequiredSubsystem(Hood subsystem0, Turret subsystem1) {
    m_hoodSubsystem = subsystem0;
    m_turretSubsystem = subsystem1;
  } 

  /**
   * Runs the Drum motor at a given speed
   * @param speed percent output form -1.0 to 1.0
   */
  public void runDrumShooter(double speed) {
    // m_shooterFalconLeft.set(TalonFXControlMode.PercentOutput, speed + speed2);
    m_shooterFalconLeft.set(speed);
    // SmartDashboard.putNumber("BoomBoom percent speed", speed + speed2);
    // SmartDashboard.putNumber("BoomBoom current stator", m_shooterFalconLeft.getStatorCurrent());
    // SmartDashboard.putNumber("BoomBoom current supply", m_shooterFalconLeft.getSupplyCurrent());

  }

  public void setShooterGains() {
    m_shooterFalconLeft.selectProfileSlot(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.SHOOTER_PID_LOOP_IDX);
    m_shooterFalconLeft.config_kF(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.kF, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconLeft.config_kP(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.kP, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconLeft.config_kI(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.kI, ShooterConstants.SHOOTER_TIMEOUT_MS);
    m_shooterFalconLeft.config_kD(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.kD, ShooterConstants.SHOOTER_TIMEOUT_MS);
  }

  public void runDrumShooterVelocityPID(double targetVel) {
    SmartDashboard.putNumber("Target Drum Velocity", 10000 + pidOffset);
    m_shooterFalconLeft.set(TalonFXControlMode.Velocity, targetVel); // Init
    
    // New BoomBoom controller stuff
    // Controls a motor with the output of the BangBang controller
    // Controls a motor with the output of the BangBang conroller and a feedforward
    // Shrinks the feedforward slightly to avoid over speeding the shooter

    // m_shooterFalconLeft.set(controller.calculate(encoder.getRate(), targetVel) + 0.9 *
    // feedforward.calculate(targetVel));
    // m_shooterFalconLeft.set(m_controller.calculate(m_shooterFalconLeft.get(), targetVel));
  }

  private static ShooterTableEntry[] readShooterTable() {
    try {
      // This is a helper class that allows us to read a CSV file into a Java array.
      CSV<ShooterTableEntry> csv = new CSV<>(ShooterTableEntry::new) {
        // This is a regular expression that removes all parentheses from the header of the CSV file.
        private final Pattern parentheses = Pattern.compile("\\([^\\)]*+\\)");

        /**
         * Removes the parentheses from the CSV header
         * 
         * @param header The header to be sanitized.
         * @return The headerSanitizer method is overriding the headerSanitizer method in the parent class.
         *         The parentheses.matcher(header) is matching the header with the parentheses regular
         *         expression. The replaceAll method is replacing all of the parentheses with an empty
         *         string. The super.headerSanitizer(parentheses.matcher(header).replaceAll("")) is calling
         *         the parent sanitizer.
         */
        @Override
        protected String headerSanitizer(String header) {
          return super.headerSanitizer(parentheses.matcher(header).replaceAll(""));
        }
      };
      // This is reading the CSV file into a Java array.
      return csv.read(new File(Filesystem.getDeployDirectory(), "ShooterData.csv").toPath());
    } catch (IOException exception) {
      ShooterTableEntry dummyEntry = new ShooterTableEntry();
      dummyEntry.distance = 0.0;
      dummyEntry.hoodExt = 0.0;
      dummyEntry.drumVelocity = 0.0;
      LOGGER.log(Level.SEVERE, "Exception while reading shooter CSV table.", exception);
      return new ShooterTableEntry[] { dummyEntry };
    }
  }

  public void updateOffset(double change) {
    pidOffset = pidOffset + change;
  }

  public void increaseSpeed(double amount)
  {
    speed2 = speed2 + amount;
  }

  public double getCurrent(){
    return m_shooterFalconLeft.getSupplyCurrent() + m_shooterFalconRight.getSupplyCurrent();
  }
}