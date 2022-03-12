// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.Vector2d;

/** Aarav's good vector class (better than WPILib) */
public class Vector2D extends Vector2d {

  public double x;
  public double y;
  public double angle;

  public Vector2D() {
    super();
    this.angle = Math.atan2(this.y, this.x);
  }

  public Vector2D(double x, double y) {
    super(x, y);

    this.x = x;
    this.y = y;
    this.angle = Math.atan2(this.y, this.x);
  }

  public static Vector2D add(Vector2D v1, Vector2D v2) {
    return new Vector2D(v1.x + v2.x, v1.y + v2.y);
  }

  public static Vector2D subtract(Vector2D v1, Vector2D v2) {
    return new Vector2D(v1.x - v2.x, v1.y - v2.y);
  }

  public static Vector2D multiply(Vector2D v1, double scalar) {
    return new Vector2D(scalar * v1.x, scalar * v1.y);
  }

  public Vector2D unit() {
    return new Vector2D(this.x / this.magnitude(), this.y / this.magnitude());
  }

  public static Vector2D round(Vector2D v, int places) {
    int scale = (int) Math.pow(10, places);

    v = Vector2D.multiply(v, scale);

    v.x = Math.round(v.x);
    v.y = Math.round(v.y);
    v.x = v.x / scale;
    v.y = v.y / scale;

    return v;
  }

  @Override
  public String toString() {
    Vector2d test = new Vector2d();
    return ("(" + this.x + ", " + this.y + ")");
  }

}
