package frc4388.utility.controller;

import edu.wpi.first.math.MathUtil;
import frc4388.robot.Constants;

public class DeadbandedRawXboxController extends RawXboxController {
  public DeadbandedRawXboxController(int port) { super(port); }
  @Override public double getLeftX() { return applyDeadband(super.getLeftX()); }
  @Override public double getLeftY() { return applyDeadband(super.getLeftY()); }
  @Override public double getRightX() { return applyDeadband(super.getRightX()); }
  @Override public double getRightY() { return applyDeadband(super.getRightY()); }
  private static double applyDeadband(double value) { return MathUtil.applyDeadband(value, Constants.OIConstants.AXIS_DEADBAND); }
}
