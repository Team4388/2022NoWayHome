package frc4388.utility.controller;

import static frc4388.robot.Constants.OIConstants.LEFT_AXIS_DEADBAND;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc4388.robot.Constants.OIConstants;

public class DeadbandedXboxController extends XboxController {
  public DeadbandedXboxController(int port) { super(port); }

  @Override public double getLeftX() { return skewToDeadzonedCircle(super.getLeftX(), super.getLeftY()).getX(); }
  @Override public double getLeftY() { return skewToDeadzonedCircle(super.getLeftX(), super.getLeftY()).getY(); }
  @Override public double getRightX() { return skewToDeadzonedCircle(super.getRightX(), super.getRightY()).getX(); }
  @Override public double getRightY() { return skewToDeadzonedCircle(super.getRightX(), super.getRightY()).getY(); }

  public int getDpadAngle() {
    return getPOV(0);
  }

  public boolean getDPadLeft(){
    return (getRawAxis(6) < -0.9);
  }

  public boolean getDPadRight(){
    return (getRawAxis(6) > 0.9);
  }

  public boolean getDPadTop(){
    return (getRawAxis(6) < 0.9);
  }

  public boolean getDPadBottom(){
    return (getRawAxis(6) > -0.9);
  }

  public static Translation2d skewToDeadzonedCircle(double x, double y) {
    Translation2d translation2d = new Translation2d(x, y);
    double magnitude = translation2d.getNorm();
    if (OIConstants.SKEW_STICKS && magnitude >= 1) return translation2d.div(magnitude);
    if (magnitude < LEFT_AXIS_DEADBAND) return new Translation2d(0,0);
    return translation2d;
  }
}
