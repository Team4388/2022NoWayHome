package frc4388.robot.commands;

import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.nio.file.attribute.FileOwnerAttributeView;
import java.nio.file.attribute.UserPrincipal;
import java.nio.file.attribute.UserPrincipalLookupService;
import java.util.Arrays;

import com.diffplug.common.base.Errors;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc4388.robot.Robot;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.BoomBoom.ShooterTableEntry;

public class ShooterTuner extends CommandBase {
  private final BoomBoom m_boomBoom;
  private final Sendable m_shotEditor;
  private final Sendable m_shotCsvAppender;
  private final Sendable m_shooterTableView;
  // private final Sendable m_shooterTableUpdater;
  private OutputStream csvOutputStream;

  public ShooterTuner(BoomBoom boomBoom) {
    m_boomBoom = boomBoom;
    addRequirements(boomBoom);
    setName("Enable");
    m_shotEditor = new ShotEditor();
    m_shotCsvAppender = new PersistentInstantCommand(this::appendCsv).withName("Append");
    m_shooterTableView = new ShooterTableEditor();
    // m_shooterTableUpdater = new
    // PersistentInstantCommand(m_boomBoom::loadShooterTable).withName("Reload");
  }

  private OutputStream getCsvOutputStream() {
    if (csvOutputStream == null) {
      Path path = new File(Filesystem.getDeployDirectory(), "ShooterData.csv").toPath();
      if (RobotBase.isReal())
        Errors.log().run(() -> Files.getFileAttributeView(path, FileOwnerAttributeView.class).setOwner(FileSystems.getDefault().getUserPrincipalLookupService().lookupPrincipalByName("admin")));
      csvOutputStream = Errors.rethrow().get(() -> Files.newOutputStream(path, StandardOpenOption.WRITE, StandardOpenOption.APPEND));
    }
    return csvOutputStream;
  }

  private class ShotEditor implements Sendable {
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("RobotPreferences");
      builder.addBooleanProperty("[Enabled]", ShooterTuner.this::isScheduled, b -> {
        if (!b) cancel();
      });
      builder.addDoubleProperty("Drum Velocity", () -> m_boomBoom.m_shooterTable[0].drumVelocity, d -> m_boomBoom.m_shooterTable[0].drumVelocity = d);
      builder.addDoubleProperty("Hood Extension", () -> m_boomBoom.m_shooterTable[0].hoodExt, d -> m_boomBoom.m_shooterTable[0].hoodExt = d);
      builder.addDoubleProperty("Measured Distance", () -> SmartDashboard.getNumber("Distance to Target", -1), System.out::println);
    }
  }

  private class ShooterTableEditor implements Sendable {
    @Override
    public void initSendable(SendableBuilder builder) {
      Arrays.stream(m_boomBoom.m_shooterTable).forEach(e -> builder.addDoubleArrayProperty(Double.toString(e.distance), () -> new double[] { e.hoodExt, e.drumVelocity }, a -> {
      }));
    }
  }

  private class PersistentInstantCommand extends InstantCommand {
    public PersistentInstantCommand(Runnable toRun, Subsystem... requirements) {
      super(toRun, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  private void appendCsv() {
    String s = String.format("%s,%s,%s%n", m_boomBoom.m_shooterTable[0].distance, m_boomBoom.m_shooterTable[0].hoodExt, m_boomBoom.m_shooterTable[0].drumVelocity);
    byte[] b = s.getBytes();
    Errors.log().run(() -> getCsvOutputStream().write(b));
  }

  @Override
  public void initialize() {
    setName("Disable");
    var tab = Shuffleboard.getTab("Shooter Tuner");
    var manual = tab.getLayout("Manual Shooter Data", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
    manual.add("Manual Shooter Data", m_shotEditor);
    manual.add("Shooter Table Appender", m_shotCsvAppender);
    var csv = tab.getLayout("Shooter Data", BuiltInLayouts.kList).withPosition(2, 0).withSize(4, 3);
    csv.add("Initial Shooter Data", m_shooterTableView);
    // csv.add("Reload Data", m_shooterTableUpdater);
    ShooterTableEntry dummyEntry = new ShooterTableEntry();
    dummyEntry.distance = 0.0;
    dummyEntry.hoodExt = 0.0;
    dummyEntry.drumVelocity = 0.0;
    m_boomBoom.m_shooterTable = new ShooterTableEntry[] { dummyEntry };
    Shuffleboard.selectTab("Shooter Tuner");
  }

  @Override
  public void execute() {
    m_boomBoom.m_shooterTable[0].distance = SmartDashboard.getNumber("Distance to Target", -1);
  }

  @Override
  public void end(boolean interrupted) {
    Errors.log().run(getCsvOutputStream()::close);
    m_boomBoom.loadShooterTable();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
