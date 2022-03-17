// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Comparator;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Pattern;
import java.util.stream.IntStream;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.CSV;
import frc4388.utility.Gains;

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

  public static class VelocityCorrectionTableEntry {
    public Double distance, duration;
  }

  private ShooterTableEntry[] m_shooterTable;
  private VelocityCorrectionTableEntry[] m_velocityCorrectionTable;

  /** Creates a new BoomBoom, which has a drum shooter and angle adjuster. */
  public BoomBoom(WPI_TalonFX shooterFalconLeft, WPI_TalonFX shooterFalconRight) {
    m_shooterFalconLeft = shooterFalconLeft;
    m_shooterFalconRight = shooterFalconRight;
    setShooterGains();

    try {
      // This is a helper class that allows us to read a CSV file into a Java array.
      CSV<ShooterTableEntry> shooterCSV = new CSV<>(ShooterTableEntry::new) {
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
        protected String headerSanitizer(final String header) {
          return super.headerSanitizer(parentheses.matcher(header).replaceAll(""));
        }
      };

      CSV<VelocityCorrectionTableEntry> velocityCorrectionCSV = new CSV<>(VelocityCorrectionTableEntry::new) {
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
        protected String headerSanitizer(final String header) {
          return super.headerSanitizer(parentheses.matcher(header).replaceAll(""));
        }
      };
      
      // This is reading the CSV file into a Java array.
      m_shooterTable = shooterCSV.read(new File(Filesystem.getDeployDirectory(), "ShooterData.csv").toPath());
      // This is a running a helper method that is logging the contents of the table to the console on a new thread.
      new Thread(() -> LOGGER.fine(() -> CSV.ReflectionTable.create(m_shooterTable, RobotBase.isSimulation()))).start();
      
      // This is reading the CSV file into a Java array.
      m_velocityCorrectionTable = velocityCorrectionCSV.read(new File(Filesystem.getDeployDirectory(), "VelocityCorrectionData.csv").toPath());
      // This is a running a helper method that is logging the contents of the table to the console on a new thread.
      new Thread(() -> LOGGER.fine(() -> CSV.ReflectionTable.create(m_velocityCorrectionTable, RobotBase.isSimulation()))).start();

    } catch (final IOException exception) {

      ShooterTableEntry dummyShooterEntry = new ShooterTableEntry();
      dummyShooterEntry.distance = 0.0;
      dummyShooterEntry.hoodExt = 0.0;
      dummyShooterEntry.drumVelocity = 0.0;
      m_shooterTable = new ShooterTableEntry[] { dummyShooterEntry };
      LOGGER.log(Level.SEVERE, "Exception while reading shooter CSV table.", exception);

      VelocityCorrectionTableEntry dummyVelocityCorrectionEntry = new VelocityCorrectionTableEntry();
      dummyVelocityCorrectionEntry.distance = 0.0;
      m_velocityCorrectionTable = new VelocityCorrectionTableEntry[] { dummyVelocityCorrectionEntry };
      LOGGER.log(Level.SEVERE, "Exception while reading velocity correction CSV table.", exception);
    }
  }

  /**
   * This is a function that takes a value (distance) and returns a value (drumVelocity) that is a
   * linear interpolation of the two values (drumVelocity) at the two closest points in the table
   * (m_shooterTable) to the given value (distance). 
   * @param distance Distance in shooter table
   * @return Drum Velocity in units per 100 ms
   */
  public Double getVelocity(final Double distance) {
    return linearInterpolate(m_shooterTable, distance, e -> e.distance, e -> e.drumVelocity).doubleValue();
  }

  /**
   * This is a function that takes a value (distance) and returns a value (hoodExt) that is a linear
   * interpolation of the two values (hoodExt) at the two closest points in the table (m_shooterTable)
   * to the given value (distance).
   * @param distance Distance in shooter table
   * @return Hood extension in units
   */
  public Double getHood(final Double distance) {
    return linearInterpolate(m_shooterTable, distance, e -> e.distance, e -> e.hoodExt).doubleValue();
  }

  /**
   * This is a function that takes a value (distance) and returns a value (duration) that is a linear
   * interpolation of the two values (duration) at the two closest points in the table (m_velocityCorrectionTable)
   * to the given value (distance).
   * @param distance Distance in velocityCorrection table
   * @return Duration in milliseconds
   */
  public Double getDuration(final Double distance) {
    return linearInterpolate(m_velocityCorrectionTable, distance, e -> e.distance, e -> e.duration).doubleValue();
  }

  /**
   * Using the given lookup value (x) and lookup getter function, locates the nearest entries in the
   * given table to be used as the lower (x0) and upper (x1) bounds for interpolation. Returns the
   * interpolation (y) between the two values (y0 and y1) accquired by applying the target getter
   * function to the lower and upper bounds entries.
   * 
   * @param table An array of entries to search through.
   * @param lookupValue The value to lookup in the table.
   * @param lookupGetter A function that takes an entry from the table and returns .
   * @param targetGetter A function that takes an E and returns a Number.
   * @return The interpolated value.
   */
  private static <E> Number linearInterpolate(final E[] table, final Number lookupValue, final Function<E, Number> lookupGetter, final Function<E, Number> targetGetter) {
    final Map.Entry<Integer, E> closestEntry = lookup(table, lookupValue.doubleValue(), lookupGetter, false).orElse(Map.entry(table.length - 1, table[table.length - 1]));
    final E closestRecord = closestEntry.getValue();
    final int closestRecordIndex = closestEntry.getKey();
    final E neighborRecord = table[lookupValue.doubleValue() <= lookupGetter.apply(closestRecord).doubleValue() ? Math.max(closestRecordIndex == 0 ? 1 : 0, closestRecordIndex - 1) : Math.min(closestRecordIndex + 1, table.length - (closestRecordIndex == table.length - 1 ? 2 : 1))];
    return lerp2(lookupValue, lookupGetter.apply(closestRecord), targetGetter.apply(closestRecord), lookupGetter.apply(neighborRecord), targetGetter.apply(neighborRecord));
  }

  /**
   * If the value is in the table, return the entry. Otherwise, return the entry with the closest
   * value
   * 
   * @param table The array of values to search.
   * @param value The value to search for.
   * @param valueGetter A function that takes an element of the table and returns a Number to compare
   *        with the given value.
   * @param exactMatch If true, the lookup will only return a match if the value is exactly equal to
   *        the value of the entry. If false, the lookup will return a match with a value closest to
   *        the given value.
   * @return The entry with the closest value to the given value.
   */
  private static <E> Optional<Map.Entry<Integer, E>> lookup(final E[] table, final Number value, final Function<E, Number> valueGetter, final boolean exactMatch) {
    final Optional<Map.Entry<Integer, E>> match = IntStream.range(0, table.length).mapToObj(i -> Map.entry(i, table[i])).min(Comparator.comparingDouble(e -> Math.abs(valueGetter.apply(e.getValue()).doubleValue() - value.doubleValue())));
    return !exactMatch || match.map(e -> valueGetter.apply(e.getValue()).equals(value)).orElse(false) ? match : Optional.empty();
  }

  /**
   * Given a value x, and two values x0 and x1, and two values y0 and y1, return a value y that is a
   * linear interpolation of the two values y0 and y1
   * 
   * @param x The value to interpolate.
   * @param x0 the x coordinate of the lower bound to interpolate to
   * @param y0 The value at x0.
   * @param x1 the x-coordinate of the upper bound to interpolate to
   * @param y1 The value at x1.
   * @return The interpolation between y0 and y1 at x.
   */
  private static Number lerp2(final Number x, final Number x0, final Number y0, final Number x1, final Number y1) {
    final Number f = (x.doubleValue() - x0.doubleValue()) / (x1.doubleValue() - x0.doubleValue());
    return (1.0 - f.doubleValue()) * y0.doubleValue() + f.doubleValue() * y1.doubleValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // speed2 = SmartDashboard.getNumber("Shooter Offset", 0.0);
    SmartDashboard.putNumber("Shooter Current", getCurrent());
    SmartDashboard.putNumber("Shooter Voltage",  m_shooterFalconLeft.getMotorOutputVoltage());
    SmartDashboard.putNumber("Shooter Actual Velocity", m_shooterFalconLeft.getSelectedSensorVelocity());
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
    m_shooterFalconLeft.set(TalonFXControlMode.PercentOutput, speed + speed2);
    SmartDashboard.putNumber("BoomBoom percent speed", speed + speed2);
    SmartDashboard.putNumber("BoomBoom current stator", m_shooterFalconLeft.getStatorCurrent());
    SmartDashboard.putNumber("BoomBoom current supply", m_shooterFalconLeft.getSupplyCurrent());

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