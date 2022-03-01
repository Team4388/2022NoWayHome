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
import java.util.regex.Pattern;
import java.util.stream.IntStream;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.utility.CSV;
import frc4388.utility.Gains;
import frc4388.utility.controller.IHandController;

public class BoomBoom extends SubsystemBase {
public WPI_TalonFX m_shooterFalconLeft;
public WPI_TalonFX m_shooterFalconRight;
public static Gains m_drumShooterGains = ShooterConstants.DRUM_SHOOTER_GAINS;
public static BoomBoom m_boomBoom;
public static IHandController m_driverController; //not sure if driverController in 2022 = m_controller in 2020
// BangBangController m_controller = new BangBangController();

double velP;
double input;

public boolean m_isDrumReady = false;
public double m_fireVel;

public Hood m_hoodSubsystem;
public Turret m_turretSubsystem;

// SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(69, 42, 0); //get real values later

public static class ShooterTableEntry {
  public Double distance, hoodExt, drumVelocity;
}

private ShooterTableEntry[] m_shooterTable;

/*
* Creates new BoomBoom subsystem, has drum shooter and angle adjuster
*/
  /** Creates a new BoomBoom. */
public BoomBoom(WPI_TalonFX shooterFalconLeft, WPI_TalonFX shooterFalconRight) {
  m_shooterFalconLeft = shooterFalconLeft;
  m_shooterFalconRight = shooterFalconRight;


  try {
    CSV<ShooterTableEntry> csv = new CSV<>(ShooterTableEntry::new) {
      private final Pattern parentheses = Pattern.compile("\\([^\\)]*+\\)");

      @Override
      protected String headerSanitizer(final String header) {
        return super.headerSanitizer(parentheses.matcher(header).replaceAll(""));
      }

    };
    m_shooterTable = csv.read(new File(Filesystem.getDeployDirectory(), "Robot Data - Distances.csv").toPath());
    new Thread(() -> System.out.println(CSV.ReflectionTable.create(m_shooterTable, RobotBase.isSimulation()))).start(); 
  } catch (final IOException e) {
    e.printStackTrace();
    // throw new RuntimeException(e);
  }
}

public Double getVelocity(final Double distance) {
  return linearInterpolate(m_shooterTable, distance, e -> e.distance, e -> e.drumVelocity).doubleValue();
}

public Double getHood(final Double distance) {
  return linearInterpolate(m_shooterTable, distance, e -> e.distance, e -> e.hoodExt).doubleValue();
}

private static <E> Number linearInterpolate(final E[] table, final Number lookupValue, final Function<E, Number> lookupGetter, final Function<E, Number> targetGetter) {
  final Map.Entry<Integer, E> closestEntry = lookup(table, lookupValue.doubleValue(), lookupGetter, false).orElse(Map.entry(table.length - 1, table[table.length - 1]));
  final E closestRecord = closestEntry.getValue();
  final int closestRecordIndex = closestEntry.getKey();
  final E neighborRecord = table[lookupValue.doubleValue() <= lookupGetter.apply(closestRecord).doubleValue() ? Math.max(closestRecordIndex == 0 ? 1 : 0, closestRecordIndex - 1) : Math.min(closestRecordIndex + 1, table.length - (closestRecordIndex == table.length - 1 ? 2 : 1))];
  return lerp2(lookupValue, lookupGetter.apply(closestRecord), targetGetter.apply(closestRecord), lookupGetter.apply(neighborRecord), targetGetter.apply(neighborRecord));
}

private static <E> Optional<Map.Entry<Integer, E>> lookup(final E[] table, final Number value, final Function<E, Number> valueGetter, final boolean exactMatch) {
  final Optional<Map.Entry<Integer, E>> match = IntStream.range(0, table.length).mapToObj(i -> Map.entry(i, table[i])).min(Comparator.comparingDouble(e -> Math.abs(valueGetter.apply(e.getValue()).doubleValue() - value.doubleValue())));
  return !exactMatch || match.map(e -> valueGetter.apply(e.getValue()).equals(value)).orElse(false) ? match : Optional.empty();
}

private static Number lerp2(final Number x, final Number x0, final Number y0, final Number x1, final Number y1) {
  final Number f = (x.doubleValue() - x0.doubleValue()) / (x1.doubleValue() - x0.doubleValue());
  return (1.0 - f.doubleValue()) * y0.doubleValue() + f.doubleValue() * y1.doubleValue();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

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
    m_shooterFalconLeft.set(TalonFXControlMode.PercentOutput, speed);
    
  }    

public void setShooterGains() {
  m_shooterFalconLeft.selectProfileSlot(ShooterConstants.SHOOTER_SLOT_IDX, ShooterConstants.SHOOTER_PID_LOOP_IDX);
  m_shooterFalconLeft.config_kF(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kF, ShooterConstants.SHOOTER_TIMEOUT_MS);
  m_shooterFalconLeft.config_kP(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kP, ShooterConstants.SHOOTER_TIMEOUT_MS);
  m_shooterFalconLeft.config_kI(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kI, ShooterConstants.SHOOTER_TIMEOUT_MS);
  m_shooterFalconLeft.config_kD(ShooterConstants.SHOOTER_SLOT_IDX, m_drumShooterGains.m_kD, ShooterConstants.SHOOTER_TIMEOUT_MS);
}

  public void runDrumShooterVelocityPID(double targetVel) {
    m_shooterFalconLeft.set(TalonFXControlMode.Velocity, targetVel); //Init
    m_shooterFalconRight.follow(m_shooterFalconLeft);
    // New BoomBoom controller stuff 
    //Controls a motor with the output of the BangBang controller
    //Controls a motor with the output of the BangBang conroller and a feedforward
    //Shrinks the feedforward slightly to avoid over speeding the shooter
    
    
    // m_shooterFalconLeft.set(controller.calculate(encoder.getRate(), targetVel) + 0.9 * feedforward.calculate(targetVel));
    // m_shooterFalconLeft.set(m_controller.calculate(m_shooterFalconLeft.get(), targetVel));
  }
}