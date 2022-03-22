// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.io.File;
import java.io.IOException;
import java.util.Comparator;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Pattern;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.CSV;

public class PlaybackDriveInput extends CommandBase {
  private static final Logger LOGGER = Logger.getLogger(PlaybackDriveInput.class.getSimpleName());

  public static class DriveInputEntry {
    public double millis, leftAxisX, leftAxisY, rightAxisX;
  }

  private DriveInputEntry[] driveInputEntries;

  private SwerveDrive swerve;
  private long startTime;

  /** Creates a new PlaybackDriveInput. */
  public PlaybackDriveInput(SwerveDrive swerve, String autoName) {
    this.swerve = swerve;

    try {
      // This is a helper class that allows us to read a CSV file into a Java array.
      CSV<DriveInputEntry> csv = new CSV<>(DriveInputEntry::new) {
        // This is a regular expression that removes all parentheses from the header of the CSV file.
        private final Pattern parentheses = Pattern.compile("\\([^\\)]*+\\)");

        /**
         * Removes the parentheses from the CSV header
         * 
         * @param header The header to be sanitized.
         * @return The headerSanitizer method is overriding the headerSanitizer method in the parent class.
         *         The parentheses.matcher(header) is matching the header with the parentheses regular
         *         expression. The replaceAll method is replacing all of the parentheses with an empty
         *         string. The super.headerSanitizer(parentheses.matcher(header).replaceAll("")) is calling
         *         the parent sanitizer.
         */
        @Override
        protected String headerSanitizer(final String header) {
          return super.headerSanitizer(parentheses.matcher(header).replaceAll(""));
        }
      };
      // This is reading the CSV file into a Java array.
      driveInputEntries = csv.read(new File(Filesystem.getDeployDirectory(), autoName + ".csv").toPath());
      // This is a running a helper method that is logging the contents of the table to the console on a new thread.
      // ! new Thread(() -> LOGGER.fine(() -> CSV.ReflectionTable.create(driveInputEntries, RobotBase.isSimulation()))).start();
    } catch (final IOException exception) {
      DriveInputEntry dummyEntry = new DriveInputEntry();
      dummyEntry.millis = 0.0;
      dummyEntry.leftAxisX = 0.0;
      dummyEntry.leftAxisY = 0.0;
      dummyEntry.rightAxisX = 0.0;
      driveInputEntries = new DriveInputEntry[] { dummyEntry };
      LOGGER.log(Level.SEVERE, "Exception while reading shooter CSV table.", exception);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    long currentTime = System.currentTimeMillis() - startTime;

    double leftAxisX = linearInterpolate(driveInputEntries, currentTime, e -> e.millis, e -> e.leftAxisX).doubleValue();
    double leftAxisY = linearInterpolate(driveInputEntries, currentTime, e -> e.millis, e -> e.leftAxisY).doubleValue();
    double rightAxisX = linearInterpolate(driveInputEntries, currentTime, e -> e.millis, e -> e.rightAxisX).doubleValue();

    swerve.driveWithInput(leftAxisX, leftAxisY, rightAxisX, true);
  }

  /**
   * Using the given lookup value (x) and lookup getter function, locates the nearest entries in the
   * given table to be used as the lower (x0) and upper (x1) bounds for interpolation. Returns the
   * interpolation (y) between the two values (y0 and y1) accquired by applying the target getter
   * function to the lower and upper bounds entries.
   * 
   * @param table An array of entries to search through.
   * @param lookupValue The value to lookup in the table.
   * @param lookupGetter A function that takes an entry from the table and returns .
   * @param targetGetter A function that takes an E and returns a Number.
   * @return The interpolated value.
   */
  private static <E> Number linearInterpolate(final E[] table, final Number lookupValue, final Function<E, Number> lookupGetter, final Function<E, Number> targetGetter) {
    final Map.Entry<Integer, E> closestEntry = lookup(table, lookupValue.doubleValue(), lookupGetter, false).orElse(Map.entry(table.length - 1, table[table.length - 1]));
    final E closestRecord = closestEntry.getValue();
    final int closestRecordIndex = closestEntry.getKey();
    final E neighborRecord = table[lookupValue.doubleValue() <= lookupGetter.apply(closestRecord).doubleValue() ? Math.max(closestRecordIndex == 0 ? 1 : 0, closestRecordIndex - 1) : Math.min(closestRecordIndex + 1, table.length - (closestRecordIndex == table.length - 1 ? 2 : 1))];
    return lerp2(lookupValue, lookupGetter.apply(closestRecord), targetGetter.apply(closestRecord), lookupGetter.apply(neighborRecord), targetGetter.apply(neighborRecord));
  }

  /**
   * If the value is in the table, return the entry. Otherwise, return the entry with the closest
   * value
   * 
   * @param table The array of values to search.
   * @param value The value to search for.
   * @param valueGetter A function that takes an element of the table and returns a Number to compare
   *        with the given value.
   * @param exactMatch If true, the lookup will only return a match if the value is exactly equal to
   *        the value of the entry. If false, the lookup will return a match with a value closest to
   *        the given value.
   * @return The entry with the closest value to the given value.
   */
  private static <E> Optional<Map.Entry<Integer, E>> lookup(final E[] table, final Number value, final Function<E, Number> valueGetter, final boolean exactMatch) {
    final Optional<Map.Entry<Integer, E>> match = IntStream.range(0, table.length).mapToObj(i -> Map.entry(i, table[i])).min(Comparator.comparingDouble(e -> Math.abs(valueGetter.apply(e.getValue()).doubleValue() - value.doubleValue())));
    return !exactMatch || match.map(e -> valueGetter.apply(e.getValue()).equals(value)).orElse(false) ? match : Optional.empty();
  }

  /**
   * Given a value x, and two values x0 and x1, and two values y0 and y1, return a value y that is a
   * linear interpolation of the two values y0 and y1
   * 
   * @param x The value to interpolate.
   * @param x0 the x coordinate of the lower bound to interpolate to
   * @param y0 The value at x0.
   * @param x1 the x-coordinate of the upper bound to interpolate to
   * @param y1 The value at x1.
   * @return The interpolation between y0 and y1 at x.
   */
  private static Number lerp2(final Number x, final Number x0, final Number y0, final Number x1, final Number y1) {
    final Number f = (x.doubleValue() - x0.doubleValue()) / (x1.doubleValue() - x0.doubleValue());
    return (1.0 - f.doubleValue()) * y0.doubleValue() + f.doubleValue() * y1.doubleValue();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
