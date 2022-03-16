// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import java.security.InvalidParameterException;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.Vector2d;

/** Aarav's good vector class (better than WPILib)
 * @author Aarav Shah */

public class Vector2D extends Vector2d {

  public double x;
  public double y;
  public double angle;

  public Vector2D() {
    this(0, 0);
  }

  public Vector2D(double x, double y) {
    super(x, y);

    this.x = x;
    this.y = y;
    this.angle = Math.atan2(this.y, this.x);
  }

  public Vector2D(double[] vec) {
    this(vec[0], vec[1]);

    if (vec.length != 2) {
      throw new InvalidParameterException();
    }
  }

  public Vector2D(Point p) {
    this(p.x, p.y);
  }

  public Vector2D(Translation2d t) {
    this(t.getX(), t.getY());
  }

  /**
   * Add two vectors, component-wise.
   * @param v1 First vector in the addition.
   * @param v2 Second vector in the addition.
   * @return New vector which is the sum.
   */
  public static Vector2D add(Vector2D v1, Vector2D v2) {
    return new Vector2D(v1.x + v2.x, v1.y + v2.y);
  }

  /**
   * Adds vector to current object
   * @param v Vector to add
   */
  public void add(Vector2D v) {
    x += v.x;
    y += v.x;
  }

  /**
   * Subtract two vectors, component-wise.
   * @param v1 First vector in the subtraction.
   * @param v2 Second vector in the subtraction.
   * @return New vector which is the difference.
   */
  public static Vector2D subtract(Vector2D v1, Vector2D v2) {
    return new Vector2D(v1.x - v2.x, v1.y - v2.y);
  }

  /**
   * Subtracts vector from current object
   * @param v Vector to subtract
   */
  public void subtract(Vector2D v) {
    x -= v.x;
    y -= v.x;
  }

  /**
   * Multiply a vector with a scalar, component-wise.
   * @param v1 Vector to multiply.
   * @param v2 Scalar to multiply.
   * @return New vector which is the product.
   */
  public static Vector2D multiply(Vector2D v1, double scalar) {
    return new Vector2D(scalar * v1.x, scalar * v1.y);
  }

  /**
   * Multiply a vector with a scalar, component-wise.
   * @param scalar Scalar to multiply
   */
  public void multiply(double scalar) {
    x *= scalar;
    y *= scalar;
  }

    /**
   * Divide a vector with a scalar, component-wise.
   * @param v1 Vector to divide.
   * @param v2 Scalar to divide.
   * @return New vector which is the division.
   */
  public static Vector2D divide(Vector2D v1, double scalar) {
    return new Vector2D(v1.x / scalar, v1.y / scalar);
  }

  /**
   * Divide a vector with a scalar, component-wise.
   * @param scalar Scalar to divide
   */
  public void divide(double scalar) {
    x /= scalar;
    y /= scalar;
  }

  /**
   * Find unit vector.
   * @return The unit vector.
   */
  public Vector2D unit() {
    return new Vector2D(this.x / this.magnitude(), this.y / this.magnitude());
  }

  /**
   * Round a vector to a certain number of places, component-wise.
   * @param v Vector to round.
   * @param places Number of places to round to.
   * @return New rounded vector.
   */
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
    return ("(" + this.x + ", " + this.y + ")");
  }

}
