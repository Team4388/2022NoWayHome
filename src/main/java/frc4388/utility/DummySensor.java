// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import java.util.ArrayList;

public class DummySensor {

  private double value;
  public double start;
  public static ArrayList<DummySensor> instances = new ArrayList<DummySensor>();

  /**
   * Creates a new DummySensor, which is a helper class for conducting PID tests without a physical robot.
   * @param init The start "position" of the sensor (default is 0).
   */
  public DummySensor(double init) {
    value = init;
    start = init;
    instances.add(this);
  }

  /**
   * Creates a new DummySensor, which is a helper class for conducting PID tests without a physical robot.
   */
  public DummySensor() {
    value = 0;
    start = 0;
    instances.add(this);
  }

  /**
   * Reset the "position" of the DummySensor to its starting value.
   */
  public void reset() {
    value = start;
  }

  /**
   * Reset the "position" of the DummySensor to a given value.
   * @param val The "position" to reset the DummySensor to.
   */
  public void reset(double val) {
    value = val;
  }

  /**
   * Reset all instances of DummySensor to their starting values.
   */
  public static void resetAll() {
    for (DummySensor instance : instances) {
      instance.reset();
    }
  }

  /**
   * Get the "position" of the DummySensor.
   * @return The current "position".
   */
  public double get() {
    return value;
  }

  /**
   * Apply an input to the DummySensor, changing its "position".
   * @param input The input to apply.
   */
  public void apply(double input) {
    value = value + input;
  }

}
