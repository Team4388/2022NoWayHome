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

public class Vector2dExt extends Vector2d {
  public Vector2dExt() {
    this(0, 0);
  }

  public Vector2dExt(double x, double y) {
    super(x, y);

    this.x = x;
    this.y = y;
  }

  public Vector2dExt(double[] vec) {
    this(vec[0], vec[1]);

    if (vec.length != 2) {
      throw new InvalidParameterException();
    }
  }

  public Vector2dExt(Point p) {
    this(p.x, p.y);
  }

  public Vector2dExt(Translation2d t) {
    this(t.getX(), t.getY());
  }

  /**
   * Add two vectors, component-wise.
   * @param v1 First vector in the addition.
   * @param v2 Second vector in the addition.
   * @return New vector which is the sum.
   */
  public static Vector2dExt add(Vector2dExt v1, Vector2dExt v2) {
    return new Vector2dExt(v1.x + v2.x, v1.y + v2.y);
  }

  /**
   * Adds vector to current object
   * @param v Vector to add
   */
  public void add(Vector2dExt v) {
    this.x += v.x;
    this.y += v.x;
  }

  /**
   * Subtract two vectors, component-wise.
   * @param v1 First vector in the subtraction.
   * @param v2 Second vector in the subtraction.
   * @return New vector which is the difference.
   */
  public static Vector2dExt subtract(Vector2dExt v1, Vector2dExt v2) {
    return new Vector2dExt(v1.x - v2.x, v1.y - v2.y);
  }

  /**
   * Subtracts vector from current object
   * @param v Vector to subtract
   */
  public void subtract(Vector2dExt v) {
    this.x -= v.x;
    this.y -= v.x;
  }

  /**
   * Multiply a vector with a scalar, component-wise.
   * @param v1 Vector to multiply.
   * @param v2 Scalar to multiply.
   * @return New vector which is the product.
   */
  public static Vector2dExt multiply(Vector2dExt v1, double scalar) {
    return new Vector2dExt(scalar * v1.x, scalar * v1.y);
  }

  /**
   * Multiply a vector with a scalar, component-wise.
   * @param scalar Scalar to multiply
   */
  public void multiply(double scalar) {
    this.x *= scalar;
    this.y *= scalar;
  }

    /**
   * Divide a vector with a scalar, component-wise.
   * @param v1 Vector to divide.
   * @param v2 Scalar to divide.
   * @return New vector which is the division.
   */
  public static Vector2dExt divide(Vector2dExt v1, double scalar) {
    return new Vector2dExt(v1.x / scalar, v1.y / scalar);
  }

  /**
   * Divide a vector with a scalar, component-wise.
   * @param scalar Scalar to divide
   */
  public void divide(double scalar) {
    this.x /= scalar;
    this.y /= scalar;
  }

  /**
   * Find unit vector.
   * @return The unit vector.
   */
  public Vector2dExt unit() {
    return new Vector2dExt(this.x / this.magnitude(), this.y / this.magnitude());
  }

  /**
   * Round a vector to a certain number of places, component-wise.
   * @param v Vector to round.
   * @param places Number of places to round to.
   * @return New rounded vector.
   */
  public static Vector2dExt round(Vector2dExt v, int places) {
    int scale = (int) Math.pow(10, places);

    v = Vector2dExt.multiply(v, scale);

    v.x = Math.round(v.x);
    v.y = Math.round(v.y);
    v.x = v.x / scale;
    v.y = v.y / scale;

    return v;
  }

  @Override
  public String toString() {
    return "<" + this.x + ", " + this.y + ">";
  }

  public double[] toDoubleArray() {
    return new double[] {this.x, this.y};
  }
}
