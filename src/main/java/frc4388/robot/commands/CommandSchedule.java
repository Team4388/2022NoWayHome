package frc4388.robot.commands;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodHandles.Lookup;
import java.lang.reflect.AccessibleObject;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.diffplug.common.base.Errors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class CommandSchedule extends CommandBase {
  private static final Logger LOGGER = Logger.getLogger(CommandSchedule.class.getSimpleName());
  private MethodHandle sequentialCommandGroupCommandsMethod;
  private MethodHandle sequentialCommandGroupCurrentCommandIndexMethod;
  private MethodHandle parallelCommandGroupCommandsMethod;
  private MethodHandle parallelDeadlineGroupCommandsMethod;
  private MethodHandle parallelRaceGroupCommandsMethod;
  private ShuffleboardLayout root;
  private ShuffleboardLayout ungroupedLayout;
  private LinkedHashMap<Command, Object> scheduledCommands;
  private final int maxWidth;
  private final int maxHeight;
  private final boolean showGroupStatus;

  public CommandSchedule(int maxWidth, int maxHeight, boolean showGroupStatus) {
    this.maxWidth = maxWidth;
    this.maxHeight = maxHeight;
    this.showGroupStatus = showGroupStatus;
  }

  @Override
  public void initialize() {
    try {
      Field scheduledCommandsField = CommandScheduler.class.getDeclaredField("m_scheduledCommands");
      Field sequentialCommandGroupCommandsField = SequentialCommandGroup.class.getDeclaredField("m_commands");
      Field sequentialCommandGroupCurrentCommandIndexField = SequentialCommandGroup.class.getDeclaredField("m_currentCommandIndex");
      Field parallelCommandGroupCommandsField = ParallelCommandGroup.class.getDeclaredField("m_commands");
      Field parallelDeadlineGroupCommandsField = ParallelDeadlineGroup.class.getDeclaredField("m_commands");
      Field parallelRaceGroupCommandsField = ParallelRaceGroup.class.getDeclaredField("m_commands");
      AccessibleObject.setAccessible(new Field[] { scheduledCommandsField, sequentialCommandGroupCommandsField, sequentialCommandGroupCurrentCommandIndexField, parallelCommandGroupCommandsField, parallelDeadlineGroupCommandsField, parallelRaceGroupCommandsField }, true);
      Lookup lookup = MethodHandles.lookup();
      sequentialCommandGroupCommandsMethod = lookup.unreflectGetter(sequentialCommandGroupCommandsField);
      sequentialCommandGroupCurrentCommandIndexMethod = lookup.unreflectGetter(sequentialCommandGroupCurrentCommandIndexField);
      parallelCommandGroupCommandsMethod = lookup.unreflectGetter(parallelCommandGroupCommandsField);
      parallelDeadlineGroupCommandsMethod = lookup.unreflectGetter(parallelDeadlineGroupCommandsField);
      parallelRaceGroupCommandsMethod = lookup.unreflectGetter(parallelRaceGroupCommandsField);
      scheduledCommands = ((LinkedHashMap<Command, Object>) lookup.unreflectGetter(scheduledCommandsField).invoke(CommandScheduler.getInstance()));
    } catch (Throwable e) {
      LOGGER.log(Level.SEVERE, "Failed to reflect necessary fields to run the command schedule.", e);
      cancel();
      return;
    }
    root = Shuffleboard.getTab("Command Schedule").getLayout("Command Schedule", BuiltInLayouts.kGrid).withSize(maxWidth, maxHeight);
    Shuffleboard.selectTab("Command Schedule");
  }

  @Override
  public void execute() {
    int size = scheduledCommands.size();
    root.withProperties(Map.of("Number of columns", size, "Number of rows", 1, "Label position", "TOP"));
    for (Command command : scheduledCommands.keySet()) {
      putCommand(command, root, size, command::isScheduled);
    }
  }

  @Override
  public void end(boolean interrupted) {
    sequentialCommandGroupCommandsMethod = null;
    sequentialCommandGroupCurrentCommandIndexMethod = null;
    parallelCommandGroupCommandsMethod = null;
    parallelDeadlineGroupCommandsMethod = null;
    parallelRaceGroupCommandsMethod = null;
    root = null;
    ungroupedLayout = null;
    scheduledCommands = null;
    purgeShuffleboardTab("Command Schedule");
  }

  @Override
  public boolean isFinished() {
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

  private void putCommand(Command command, ShuffleboardContainer layout, int siblings, BooleanSupplier running) {
    boolean isRoot = root == layout;
    String name = command.getName() + "@" + Integer.toHexString(command.hashCode());
    if (command instanceof CommandGroupBase) {
      Collection<Command> commands = List.of();
      Function<Command, BooleanSupplier> nestedRunningMaker = c -> () -> !c.isFinished();
      if (command instanceof SequentialCommandGroup) {
        ArrayList<Command> commandsList = Errors.log().getWithDefault(() -> (ArrayList<Command>) sequentialCommandGroupCommandsMethod.invoke(command), new ArrayList<>());
        commands = commandsList;
        nestedRunningMaker = c -> () -> Errors.log().getWithDefault(() -> (int) sequentialCommandGroupCurrentCommandIndexMethod.invoke(command) == commandsList.indexOf(c), false);
      } else if (command instanceof ParallelCommandGroup) {
        HashMap<Command, Boolean> commandsMap = Errors.log().getWithDefault(() -> (HashMap<Command, Boolean>) parallelCommandGroupCommandsMethod.invoke(command), new HashMap<Command, Boolean>());
        commands = commandsMap.keySet();
        nestedRunningMaker = c -> () -> commandsMap.get(c);
      } else if (command instanceof ParallelDeadlineGroup) {
        commands = Errors.log().getWithDefault(() -> (HashMap<Command, Boolean>) parallelDeadlineGroupCommandsMethod.invoke(command), new HashMap<Command, Boolean>()).keySet();
        nestedRunningMaker = c -> () -> !command.isFinished();
      } else if (command instanceof ParallelRaceGroup) {
        commands = Errors.log().getWithDefault(() -> (HashSet<Command>) parallelRaceGroupCommandsMethod.invoke(command), new HashSet<>());
        nestedRunningMaker = c -> () -> !command.isFinished();
      }
      ShuffleboardLayout nestedLayout;
      int size = commands.size() + (showGroupStatus ? 1 : 0);
      if (isRoot)
        nestedLayout = layout.getLayout(name, BuiltInLayouts.kList).withSize(maxWidth / siblings, maxHeight);
      else
        nestedLayout = layout.getLayout(name, BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", size, "Number of rows", 1));
      if (showGroupStatus && nestedLayout.getComponents().stream().map(ShuffleboardComponent::getTitle).noneMatch("_self_"::equals))
        nestedLayout.addBoolean("_self_", running);
      for (Command nestedCommand : commands) {
        putCommand(nestedCommand, nestedLayout, size, nestedRunningMaker.apply(nestedCommand));
      }
    } else if (command instanceof CommandBase) {
      ShuffleboardContainer target = isRoot ? Objects.requireNonNullElseGet(ungroupedLayout, () -> ungroupedLayout = root.getLayout("Ungrouped", BuiltInLayouts.kList)) : layout;
      if (target.getComponents().stream().map(ShuffleboardComponent::getTitle).noneMatch(name::equals))
        target.addBoolean(name, running);
    }
  }

  private static void purgeShuffleboardTab(String name) {
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
    } catch (NoSuchFieldException | IllegalAccessException e) {
      LOGGER.log(Level.SEVERE, "Failed to purge Shuffleboard tab " + name + ".", e);
    }
    Shuffleboard.update();
  }

  private static void recursiveClearTable(NetworkTable table) {
    table.getSubTables().forEach(name -> recursiveClearTable(table.getSubTable(name)));
    table.getSubTables().forEach(table::delete);
    table.getKeys().forEach(table::delete);
  }
}
