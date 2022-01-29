package frc4388.utility.controller;

import static frc4388.robot.Constants.OIConstants.AXIS_DEADBAND;

import edu.wpi.first.math.geometry.Translation2d;

public class DeadbandedRawXboxController extends RawXboxController {
  public DeadbandedRawXboxController(int port) { super(port); }
  @Override public double getLeftX() { return skewToDeadzonedCircle(super.getLeftX(), super.getLeftY()).getX(); }
  @Override public double getLeftY() { return skewToDeadzonedCircle(super.getLeftX(), super.getLeftY()).getY(); }
  @Override public double getRightX() { return skewToDeadzonedCircle(super.getRightX(), super.getRightY()).getX(); }
  @Override public double getRightY() { return skewToDeadzonedCircle(super.getRightX(), super.getRightY()).getY(); }
  public static Translation2d skewToDeadzonedCircle(double x, double y) {
    Translation2d translation2d = new Translation2d(x, y);
    double magnitude = translation2d.getNorm();
    if (magnitude >= 1) return translation2d.div(magnitude);
    if (magnitude < AXIS_DEADBAND) return new Translation2d(0,0);
    return translation2d;
  }
}
