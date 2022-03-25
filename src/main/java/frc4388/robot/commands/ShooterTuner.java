package frc4388.robot.commands;

import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.nio.file.attribute.FileOwnerAttributeView;
import java.util.Arrays;
import java.util.logging.Logger;

import com.diffplug.common.base.Errors;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.BoomBoom.ShooterTableEntry;

public class ShooterTuner extends CommandBase {
  private static final Logger LOGGER = Logger.getLogger(ShooterTuner.class.getSimpleName());
  private static final Path PATH = new File(Filesystem.getDeployDirectory(), "ShooterData.csv").toPath();
  private final BoomBoom m_boomBoom;
  private final ShotEditor m_shotEditor;
  private final CSVAppender m_shotCsvAppender;
  private final DistanceReader m_distanceReader;
  private final ShooterTableEditor m_shooterTableEditor;
  private final DisabledInstantCommand m_shooterTableUpdater;
  private final DisabledInstantCommand m_printCsvFile;
  private final ShooterTableEntry tableOverrideEntry;

  public ShooterTuner(BoomBoom boomBoom) {
    m_boomBoom = boomBoom;
    m_shotEditor = new ShotEditor();
    m_shotCsvAppender = new CSVAppender();
    m_distanceReader = new DistanceReader();
    m_shooterTableEditor = new ShooterTableEditor();
    m_printCsvFile = new DisabledInstantCommand(() -> LOGGER.info(Errors.log().wrapWithDefault(() -> Files.readString(PATH), "Failed to read CSV")), "Print");
    m_shooterTableUpdater = new DisabledInstantCommand(m_boomBoom::loadShooterTable, "Load CSV");
    tableOverrideEntry = new ShooterTableEntry();
    setName("Shooter Data Mode");
  }

  @Override
  public void initialize() {
    var tab = Shuffleboard.getTab("Shooter Tuner");
    if (tab.getComponents().isEmpty()) {
      var manual = tab.getLayout("Manual Shooter Data", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
      manual.add("Distance Reader", m_distanceReader);
      manual.add("Manual Shooter Data", m_shotEditor);
      manual.add("Shooter Table Appender", m_shotCsvAppender);
      var csv = tab.getLayout("Shooter Data", BuiltInLayouts.kList).withPosition(2, 0).withSize(4, 3);
      csv.addBoolean("Is Shooter Data Overridden", this::isOverridden);
      csv.add("Shooter Data (Broken)", m_shooterTableEditor);
      csv.add("Shooter CSV Loader", m_shooterTableUpdater);
      csv.add("Print CSV File", m_printCsvFile);
    }
    tableOverrideEntry.distance = 0.0;
    tableOverrideEntry.hoodExt = 0.0;
    tableOverrideEntry.drumVelocity = 0.0;
    m_boomBoom.m_shooterTable = new ShooterTableEntry[] { tableOverrideEntry };
    Shuffleboard.selectTab("Shooter Tuner");
  }

  @Override
  public final boolean isFinished() {
    return true;
  }

  @Override
  public String getName() {
    return isOverridden() ? "Tuner Override" : "CSV File";
  }

  private boolean isOverridden() {
    return m_boomBoom.m_shooterTable.length == 1 && m_boomBoom.m_shooterTable[0].equals(tableOverrideEntry);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  private static void setReadOnlyProperty(Object o) {
    System.err.println("Unable to set read-only property.");
  }

  private class DisabledInstantCommand extends InstantCommand {
    public DisabledInstantCommand(Runnable toRun, String name, Subsystem... requirements) {
      super(toRun, requirements);
      setName(name);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  private class DistanceReader extends CommandBase {
    @Override
    public void execute() {
      tableOverrideEntry.distance = SmartDashboard.getNumber("Distance to Target", -1);
    }

    @Override
    public String getName() {
      return isScheduled() ? "Reading" : "Enable";
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  private class ShotEditor implements Sendable {
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("RobotPreferences");
      builder.addDoubleProperty("Drum Velocity", () -> tableOverrideEntry.drumVelocity, d -> tableOverrideEntry.drumVelocity = d);
      builder.addDoubleProperty("Hood Extension", () -> tableOverrideEntry.hoodExt, d -> tableOverrideEntry.hoodExt = d);
      builder.addDoubleProperty("Measured Distance", () -> tableOverrideEntry.distance, ShooterTuner::setReadOnlyProperty);
    }
  }

  private class CSVAppender extends CommandBase {
    @Override
    public void initialize() {
      if (RobotBase.isReal()) Errors.log().run(() -> Files.getFileAttributeView(PATH, FileOwnerAttributeView.class).setOwner(FileSystems.getDefault().getUserPrincipalLookupService().lookupPrincipalByName("admin")));
      try (OutputStream csvOutputStream = Files.newOutputStream(PATH, StandardOpenOption.WRITE, StandardOpenOption.APPEND)) {
        csvOutputStream.write(String.format("%s,%s,%s%n", tableOverrideEntry.distance, tableOverrideEntry.hoodExt, tableOverrideEntry.drumVelocity).getBytes());
      } catch (IOException e) {
        System.out.println(e);
      }
    }

    @Override
    public String getName() {
      return isScheduled() ? "Appending" : "Append";
    }

    @Override
    public final boolean isFinished() {
      return true;
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  private class ShooterTableEditor implements Sendable {
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addStringArrayProperty("distance", () -> new String[] { "hoodExt", "drumVelocity" }, ShooterTuner::setReadOnlyProperty);
      Arrays.stream(m_boomBoom.m_shooterTable).forEach(e -> builder.addDoubleArrayProperty(Double.toString(e.distance), () -> new double[] { e.hoodExt, e.drumVelocity }, ShooterTuner::setReadOnlyProperty));
    }
  }
}
