// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;

public class RecordDriveInput extends CommandBase {
  private HashMap<Long, double[]> timedInput;

  private SwerveDrive swerve;

  private Supplier<Double> leftAxisXSupplier;
  private Supplier<Double> leftAxisYSupplier;
  private Supplier<Double> rightAxisXSupplier;

  private String autoName;

  private long startTime;
  private long count;

  private int nthRecord;

  /** Creates a new RecordDriveInput. */
  public RecordDriveInput(
        SwerveDrive swerve,
        Supplier<Double> leftAxisXSupplier,
        Supplier<Double> leftAxisYSupplier,
        Supplier<Double> rightAxisXSupplier,
        String autoName,
        int nthRecord)
  {
    this.swerve = swerve;
    this.leftAxisXSupplier = leftAxisXSupplier;
    this.leftAxisYSupplier = leftAxisYSupplier;
    this.rightAxisXSupplier = rightAxisXSupplier;
    this.autoName = autoName;
    this.nthRecord = nthRecord;

    timedInput = new HashMap<>();

    addRequirements(this.swerve);
  }

  public RecordDriveInput(
        SwerveDrive swerve,
        Supplier<Double> leftAxisXSupplier,
        Supplier<Double> leftAxisYSupplier,
        Supplier<Double> rightAxisXSupplier,
        String autoName)
  {
    this(swerve, leftAxisXSupplier, leftAxisYSupplier, rightAxisXSupplier, autoName, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    count = 0;

    timedInput.put((long) 0, new double[] {0, 0, 0});
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    long timeFromStart = System.currentTimeMillis() - startTime;
    double[] input = {leftAxisXSupplier.get(), leftAxisYSupplier.get(), rightAxisXSupplier.get()};

    if(count % (long) nthRecord == 0)
      timedInput.put(timeFromStart, input);

    swerve.driveWithInput(input[0], input[1], input[2], true);
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    File csvOutput = new File(autoName + ".csv");
    try(PrintWriter writer = new PrintWriter(csvOutput)) {
      writer.println("millis,leftx,lefty,rightx");

      for(long millis : timedInput.keySet())
        writer.println(millis + "," + timedInput.get(millis)[0] + "," + timedInput.get(millis)[1] + "," + timedInput.get(millis)[2]);

      writer.println((millis + 1) + ",0,0,0");
      writer.close();
    } catch(Exception e) {
      e.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}