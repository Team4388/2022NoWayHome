package frc4388.robot.commands.shuffleboard;

import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.nio.file.attribute.FileOwnerAttributeView;
import java.util.logging.Level;
import java.util.logging.Logger;

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
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.BoomBoom.ShooterTableEntry;
import frc4388.utility.shuffleboard.SendableTable;
import frc4388.utility.shuffleboard.ShuffleboardHelper;

public class ShooterTuner extends CommandBase {
  private static final Logger LOGGER = Logger.getLogger(ShooterTuner.class.getSimpleName());
  private static final Path PATH = new File(Filesystem.getDeployDirectory(), "ShooterData.csv").toPath();
  private final BoomBoom m_boomBoom;
  private final ShotEditor m_shotEditor;
  private final CSVAppender m_shotCsvAppender;
  private final ShooterTableEntry tableOverrideEntry;
  private final SendableTable m_tableEditor;
  private boolean measureDistance = false;
  private boolean triedTakeOwnership = false;

  public ShooterTuner(BoomBoom boomBoom) {
    m_boomBoom = boomBoom;
    m_shotEditor = new ShotEditor();
    m_shotCsvAppender = new CSVAppender();
    tableOverrideEntry = new ShooterTableEntry();
    m_tableEditor = new SendableTable(m_boomBoom::getShooterTable, m_boomBoom::setShooterTable);
    setName("Shooter Data Mode");
  }

  @Override
  public void initialize() {
    var tab = Shuffleboard.getTab("Shooter Tuner");
    if (tab.getComponents().isEmpty()) {
      var manual = tab.getLayout("Manual Shooter Data", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 5);
      manual.add("Manual Shooter Data", m_shotEditor);
      manual.add("Manual Data Appender", m_shotCsvAppender);
      var csv = tab.getLayout("Shooter Table", BuiltInLayouts.kList).withPosition(2, 0).withSize(7, 5);
      csv.add("Shooter Table", m_tableEditor);
      csv.add("Save to CSV File", new InstantCommand(m_boomBoom::saveShooterTable) {
        @Override
        public boolean runsWhenDisabled() {
          return true;
        }
      }.withName("DO NOT RUN WHILE TUNER IS ENABLED"));
      csv.add("Shooter Tuner State (Disable to Reload)", this);
    }
    tableOverrideEntry.distance = 0.0;
    tableOverrideEntry.hoodExt = 0.0;
    tableOverrideEntry.drumVelocity = 0.0;
    tableOverrideEntry.turretOffset = 0.0;
    m_boomBoom.setShooterTable(new ShooterTableEntry[] { tableOverrideEntry });
    Shuffleboard.selectTab("Shooter Tuner");
    SmartDashboard.putData("Shooter Table", m_tableEditor);
  }
  @Override
  public void execute() {
    if (measureDistance)
      tableOverrideEntry.distance = SmartDashboard.getNumber("SmartDashboard/Distance to Target", -1);
  }
  @Override
  public void end(boolean interrupted) {
    m_boomBoom.loadShooterTable();
    LOGGER.info(Errors.log().wrapWithDefault(() -> Files.readString(PATH), "Failed to read CSV"));
    // ShuffleboardHelper.purgeShuffleboardTab("Shooter Tuner");
  }
  @Override
  public final boolean isFinished() {
    return false;
  }

  @Override
  public String getName() {
    return isScheduled() ? "Enabled" : "Disabled";
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  private class ShotEditor implements Sendable {
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("RobotPreferences");
      builder.addDoubleProperty("Drum Velocity", () -> tableOverrideEntry.drumVelocity, d -> tableOverrideEntry.drumVelocity = d);
      builder.addDoubleProperty("Hood Extension", () -> tableOverrideEntry.hoodExt, d -> tableOverrideEntry.hoodExt = d);
      builder.addDoubleProperty("Turret Offset", () -> tableOverrideEntry.turretOffset, d -> tableOverrideEntry.turretOffset = d);
      builder.addDoubleProperty("Distance", () -> tableOverrideEntry.distance, d -> tableOverrideEntry.distance = d);
      builder.addBooleanProperty("Measure Distance", () -> measureDistance, b -> measureDistance = b);
    }
  }

  private class CSVAppender extends CommandBase {
    @Override
    public void execute() {
      if (!triedTakeOwnership && RobotBase.isReal()) {
        triedTakeOwnership = true;
        Errors.log().run(() -> Files.getFileAttributeView(PATH, FileOwnerAttributeView.class).setOwner(FileSystems.getDefault().getUserPrincipalLookupService().lookupPrincipalByName("admin")));
      }
      try (OutputStream csvOutputStream = Files.newOutputStream(PATH, StandardOpenOption.WRITE, StandardOpenOption.APPEND)) {
        csvOutputStream.write(String.format("%s,%s,%s%n", tableOverrideEntry.distance, tableOverrideEntry.hoodExt, tableOverrideEntry.drumVelocity).getBytes());
      } catch (IOException e) {
        LOGGER.log(Level.SEVERE, "Failed to write CSV", e);
      }
      super.cancel();
    }

    @Override
    public String getName() {
      return isScheduled() ? "Appending" : "Append to File";
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }
}
