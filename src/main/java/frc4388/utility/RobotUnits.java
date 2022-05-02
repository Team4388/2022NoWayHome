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
