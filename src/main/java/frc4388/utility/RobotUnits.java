// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class RobotUnits {
  // constants

  // conversions
  public static double falconTicksToRotations(final double ticks) {
    double rotations = ticks / 2048;
    return rotations;
  }

  public static double falconRotationsToTicks(final double rotations) {
    double ticks = rotations * 2048;
    return ticks;
  }
}
