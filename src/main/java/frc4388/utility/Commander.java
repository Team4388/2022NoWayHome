package frc4388.utility;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.reflect.AccessibleObject;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.diffplug.common.base.Errors;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Commander {
  private static MethodHandle m_scheduledCommandsHandle;
  private static MethodHandle m_sequentialCommandGroupCommandsHandle;
  private static MethodHandle m_sequentialCommandGroupCurrentCommandIndexHandle;
  private static MethodHandle m_parallelCommandGroupCommandsHandle;
  private static MethodHandle m_parallelDeadlineGroupCommandsHandle;
  private static MethodHandle m_parallelRaceGroupCommandsHandle;
  private static ShuffleboardTab tab;

  public static void initialize() {
    try {
      Field m_scheduledCommandsField = CommandScheduler.class.getDeclaredField("m_scheduledCommands");
      m_scheduledCommandsField.trySetAccessible();
      m_scheduledCommandsHandle = MethodHandles.lookup().unreflectGetter(m_scheduledCommandsField);
      Field m_sequentialCommandGroupCommandsField = SequentialCommandGroup.class.getDeclaredField("m_commands");
      m_sequentialCommandGroupCommandsField.trySetAccessible();
      m_sequentialCommandGroupCommandsHandle = MethodHandles.lookup().unreflectGetter(m_sequentialCommandGroupCommandsField);
      Field m_sequentialCommandGroupCurrentCommandIndexField = SequentialCommandGroup.class.getDeclaredField("m_currentCommandIndex");
      m_sequentialCommandGroupCurrentCommandIndexField.trySetAccessible();
      m_sequentialCommandGroupCurrentCommandIndexHandle = MethodHandles.lookup().unreflectGetter(m_sequentialCommandGroupCurrentCommandIndexField);
      Field m_parallelCommandGroupCommandsField = ParallelCommandGroup.class.getDeclaredField("m_commands");
      m_parallelCommandGroupCommandsField.trySetAccessible();
      m_parallelCommandGroupCommandsHandle = MethodHandles.lookup().unreflectGetter(m_parallelCommandGroupCommandsField);
      Field m_parallelDeadlineGroupCommandsField = ParallelDeadlineGroup.class.getDeclaredField("m_commands");
      m_parallelDeadlineGroupCommandsField.trySetAccessible();
      m_parallelDeadlineGroupCommandsHandle = MethodHandles.lookup().unreflectGetter(m_parallelDeadlineGroupCommandsField);
      Field m_parallelRaceGroupCommandsField = ParallelRaceGroup.class.getDeclaredField("m_commands");
      m_parallelRaceGroupCommandsField.trySetAccessible();
      m_parallelRaceGroupCommandsHandle = MethodHandles.lookup().unreflectGetter(m_parallelRaceGroupCommandsField);
    } catch (IllegalArgumentException | NoSuchFieldException | SecurityException | IllegalAccessException e) {
      e.printStackTrace();
    }
    tab = Shuffleboard.getTab("Commander");
    try {
      scheduledCommands = ((LinkedHashMap<Command, Object>) m_scheduledCommandsHandle.invoke(CommandScheduler.getInstance()));
    } catch (Throwable e) {
      e.printStackTrace();
    }
  }
  private static LinkedHashMap<Command, Object> scheduledCommands;

  public static void periodic() {
    int count = scheduledCommands.size();
    for (Command command : scheduledCommands.keySet()) {
      putCommand(command, null, count, command::isScheduled);
    }
  }
  private static int count = 0;
  private static void putCommand(Command command, ShuffleboardContainer layout, int siblings, BooleanSupplier running) {
    String name = (count++) + ":" + command.getClass().getSimpleName()/*  + "@" + Integer.toHexString(command.hashCode()) */;
    if (Objects.requireNonNullElse(layout, tab).getComponents().stream().map(ShuffleboardComponent::getTitle).anyMatch(name::equals)) return;
    if (command instanceof CommandGroupBase) {
      Collection<Command> commands = List.of();
      String glyph = "CUBE";
      Function<Command, BooleanSupplier> nestedRunningMaker = c -> () -> !c.isFinished();
      if (command instanceof SequentialCommandGroup) {
        ArrayList<Command> commandsList = Errors.log().getWithDefault(() -> (ArrayList<Command>) m_sequentialCommandGroupCommandsHandle.invoke(command), new ArrayList<>());
        commands = commandsList;
        nestedRunningMaker = c -> () -> Errors.log().getWithDefault(() -> (int) m_sequentialCommandGroupCurrentCommandIndexHandle.invoke(command) == commandsList.indexOf(c), false);
        glyph = "LIST_OL";
      } else if (command instanceof ParallelCommandGroup) {
        HashMap<Command, Boolean> commandsMap = Errors.log().getWithDefault(() -> (HashMap<Command, Boolean>) m_parallelCommandGroupCommandsHandle.invoke(command), new HashMap<Command, Boolean>());
        commands = commandsMap.keySet();
        nestedRunningMaker = c -> () -> commandsMap.get(c);
        glyph = "LIST_UL";
      } else if (command instanceof ParallelDeadlineGroup) {
        HashMap<Command, Boolean> commandsMap = Errors.log().getWithDefault(() -> (HashMap<Command, Boolean>) m_parallelDeadlineGroupCommandsHandle.invoke(command), new HashMap<Command, Boolean>());
        commands = commandsMap.keySet();
        nestedRunningMaker = c -> () -> commandsMap.get(c);
        glyph = "CROSSHAIRS";
      } else if (command instanceof ParallelRaceGroup) {
        commands = Errors.log().getWithDefault(() -> (HashSet<Command>) m_parallelRaceGroupCommandsHandle.invoke(command), new HashSet<>());
        nestedRunningMaker = c -> () -> !command.isFinished();
        glyph = "RANDOM";
      }
      ShuffleboardLayout nestedLayout = Objects.requireNonNullElse(layout, tab).getLayout(name, layout == null ? BuiltInLayouts.kList : BuiltInLayouts.kGrid).withSize(11 / siblings, layout == null ? 6 : 1).withProperties(Map.of("Number of columns", 11 / siblings, "Number of rows", 1, "Label position", "BOTTOM", "Show Glyph", true, "Glyph", glyph));
      int count = commands.size();
      for (Command nestedCommand : commands) {
        putCommand(nestedCommand, nestedLayout, count, nestedRunningMaker.apply(nestedCommand));
      }
    } else if (command instanceof CommandBase) {
      Objects.requireNonNullElse(layout, tab).addBoolean(name, running);
    }
  }
}
