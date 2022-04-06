package frc4388.utility.shuffleboard;

import java.lang.reflect.Field;
import java.util.LinkedHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class ShuffleboardHelper {
  private static final Logger LOGGER = Logger.getLogger(ShuffleboardHelper.class.getSimpleName());

  @SuppressWarnings("unchecked")
  public static void purgeShuffleboardTab(String name) {
    Shuffleboard.getTab(name).getComponents().clear();
    NetworkTable rootTable = NetworkTableInstance.getDefault().getTable("Shuffleboard");
    NetworkTable rootMetaTable = rootTable.getSubTable(".metadata");
    recursiveClearTable(rootMetaTable.getSubTable(name));
    recursiveClearTable(rootTable.getSubTable(name));
    rootMetaTable.getEntry("Selected").setString("");
    rootMetaTable.delete(name);
    rootTable.delete(name);
    try {
      Field shuffleboardRootField = Shuffleboard.class.getDeclaredField("root");
      shuffleboardRootField.trySetAccessible();
      Object shuffleboardRoot = shuffleboardRootField.get(null);
      Field shuffleboardTabsField = shuffleboardRoot.getClass().getDeclaredField("m_tabs");
      Field shuffleboardTabsChangedField = shuffleboardRoot.getClass().getDeclaredField("m_tabsChanged");
      shuffleboardTabsField.trySetAccessible();
      shuffleboardTabsChangedField.trySetAccessible();
      ((LinkedHashMap<String, ShuffleboardTab>) shuffleboardTabsField.get(shuffleboardRoot)).remove(name);
      shuffleboardTabsChangedField.set(shuffleboardRoot, true);
    } catch (NoSuchFieldException | IllegalAccessException | ClassCastException exception) {
      LOGGER.log(Level.SEVERE, exception, () -> "Failed to purge Shuffleboard tab " + name + ".");
    }
    Shuffleboard.update();
  }

  public static void recursiveClearTable(NetworkTable table) {
    table.getSubTables().forEach(name -> recursiveClearTable(table.getSubTable(name)));
    table.getSubTables().forEach(table::delete);
    table.getKeys().forEach(table::delete);
  }
}
