package frc4388.utility;

import static org.fusesource.jansi.Ansi.ansi;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.StreamHandler;

import com.diffplug.common.base.DurianPlugins;
import com.diffplug.common.base.Errors;

import org.fusesource.jansi.Ansi.Attribute;
import org.fusesource.jansi.Ansi.Color;
import org.fusesource.jansi.AnsiConsole;
import org.fusesource.jansi.AnsiPrintStream;

import edu.wpi.first.hal.HAL;
import javassist.CannotCompileException;
import javassist.ClassPool;
import javassist.CtClass;
import javassist.NotFoundException;

public final class AnsiLogging {
  public static final Level LEVEL = Level.ALL;
  private static final AnsiPrintStream ANSI_CONSOLE_STREAM = AnsiConsole.err();

  public static Handler halLoggerHandler = null;
  public static void systemInstall() {
    try {
      // Configure java.util.logging.Logger to output additional colored information.
      LogManager.getLogManager().updateConfiguration(key -> (o, n) -> {
        switch (key) {
          case ".level":
            return LEVEL.getName();
          case "handlers":
            return LoggingAnsiConsoleHandler.class.getName();
          default:
            return n;
        }
      });
      // Set the console to process ANSI escape codes.
      ANSI_CONSOLE_STREAM.install();
      // Sends standard output stream messages through a logger.
      System.setOut(printStreamLogger(false, s -> Logger.getGlobal().logp(Level.INFO, null, "out", s)));
      // Sends standard error output stream messages through a logger.
      System.setErr(printStreamLogger(false, s -> Logger.getGlobal().logp(Level.SEVERE, null, "err", s)));
      // This is registering a plugin that will log Durian errors to the console using a logger.
      DurianPlugins.register(Errors.Plugins.Log.class, e -> Logger.getLogger(e.getStackTrace()[0].getClassName().substring(e.getStackTrace()[0].getClassName().lastIndexOf('.') + 1)).log(Level.SEVERE, e, e::getLocalizedMessage));
      // Store the handler for HAL to use when sending errors to DriverStation.
      halLoggerHandler = new LoggingAnsiConsoleHandler(printStreamLogger(true, s -> HAL.sendError(false, 0, false, s, "", "", true)));
      // Use Javassist to replace the DriverStation class bytecode with our own.
      ClassPool classPool = ClassPool.getDefault();
      CtClass ctClassDriverStation = classPool.get("edu.wpi.first.wpilibj.DriverStation");
      ctClassDriverStation.getDeclaredMethod("reportErrorImpl", new CtClass[] { CtClass.booleanType, CtClass.intType, classPool.get("java.lang.String"), CtClass.booleanType, classPool.get("java.lang.StackTraceElement[]"), CtClass.intType }).setBody("{boolean isError=$1;int code=$2;String error=$3;boolean printTrace=$4;StackTraceElement[] stackTrace=$5;int stackTraceFirst=$6;if(frc4388.utility.AnsiLogging.halLoggerHandler!=null){if(!frc4388.utility.AnsiLogging.LEVEL.equals(java.util.logging.Level.OFF)){java.util.logging.LogRecord logRecord=new java.util.logging.LogRecord(isError?java.util.logging.Level.SEVERE:java.util.logging.Level.FINER,error.stripTrailing());logRecord.setLoggerName(\"HAL\");if(stackTrace!=null&&stackTrace.length>=stackTraceFirst+1){int from=Math.min(Math.max(0,stackTraceFirst),stackTrace.length-1);java.lang.StackTraceElement[] presentStackTrace=new java.lang.StackTraceElement[stackTrace.length-from-1];System.arraycopy(stackTrace,from,presentStackTrace,0,presentStackTrace.length);logRecord.setSourceMethodName(presentStackTrace[0].getMethodName());boolean isEpochs=!printTrace&&presentStackTrace[0].toString().equals(\"edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63)\");if(printTrace||isEpochs){String throwableMessage=\"\";if(isEpochs){throwableMessage=\"Epochs\"+System.lineSeparator()+logRecord.getMessage();presentStackTrace=new java.lang.StackTraceElement[0];logRecord.setLevel(java.util.logging.Level.FINEST);logRecord.setMessage(\"Execution times:\");}else{long lineCount=logRecord.getMessage().lines().count();}java.lang.Throwable throwable=new java.lang.Throwable(throwableMessage);throwable.setStackTrace(presentStackTrace);logRecord.setThrown(throwable);}}if(!frc4388.utility.AnsiLogging.halLoggerHandler.isLoggable(logRecord))return;frc4388.utility.AnsiLogging.halLoggerHandler.publish(logRecord);}}else{String locString=\"\";if(stackTrace.length>=stackTraceFirst+1){locString=stackTrace[stackTraceFirst].toString();}else{locString=\"\";}StringBuilder traceString=new StringBuilder();if(printTrace){boolean haveLoc=false;for(int i=stackTraceFirst;i<stackTrace.length;i++){String loc=stackTrace[i].toString();traceString.append(\"\\tat \").append(loc).append('\\n');if(!haveLoc&&!loc.startsWith(\"edu.wpi.first\")){locString=loc;haveLoc=true;}}}edu.wpi.first.hal.HAL.sendError(isError,code,false,error,locString,traceString.toString(),true);}}");
      /*
      {
        boolean isError = $1;
        int code = $2;
        String error = $3;
        boolean printTrace = $4;
        StackTraceElement[] stackTrace = $5;
        int stackTraceFirst = $6;
        if (frc4388.utility.AnsiLogging.halLoggerHandler != null) {
          if (!frc4388.utility.AnsiLogging.LEVEL.equals(java.util.logging.Level.OFF)) {
            java.util.logging.LogRecord logRecord = new java.util.logging.LogRecord(isError ? java.util.logging.Level.SEVERE : java.util.logging.Level.FINER, error.stripTrailing());
            logRecord.setLoggerName("HAL");
            if (stackTrace != null && stackTrace.length >= stackTraceFirst + 1) {
              int from = Math.min(Math.max(0, stackTraceFirst), stackTrace.length - 1);
              java.lang.StackTraceElement[] presentStackTrace = new java.lang.StackTraceElement[stackTrace.length - from - 1];
              System.arraycopy(stackTrace, from, presentStackTrace, 0, presentStackTrace.length);
              logRecord.setSourceMethodName(presentStackTrace[0].getMethodName());
              boolean isEpochs = !printTrace && presentStackTrace[0].toString().equals("edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63)");
              if (printTrace || isEpochs) {
                String throwableMessage = "";
                if (isEpochs) {
                  throwableMessage = "Epochs" + System.lineSeparator() + logRecord.getMessage();
                  presentStackTrace = new java.lang.StackTraceElement[0];
                  logRecord.setLevel(java.util.logging.Level.FINEST);
                  logRecord.setMessage("Execution times:");
                } else {
                  long lineCount = logRecord.getMessage().lines().count();
                }
                java.lang.Throwable throwable = new java.lang.Throwable(throwableMessage);
                throwable.setStackTrace(presentStackTrace);
                logRecord.setThrown(throwable);
              }
            }
            if (!frc4388.utility.AnsiLogging.halLoggerHandler.isLoggable(logRecord)) return;
            frc4388.utility.AnsiLogging.halLoggerHandler.publish(logRecord);
          }
        } else {
          String locString = "";
          if (stackTrace.length >= stackTraceFirst + 1) {
            locString = stackTrace[stackTraceFirst].toString();
          } else {
            locString = "";
          }
          StringBuilder traceString = new StringBuilder();
          if (printTrace) {
            boolean haveLoc = false;
            for (int i = stackTraceFirst; i < stackTrace.length; i++) {
              String loc = stackTrace[i].toString();
              traceString.append("\tat ").append(loc).append('\n');
              // get first user function
              if (!haveLoc && !loc.startsWith("edu.wpi.first")) {
                locString = loc;
                haveLoc = true;
              }
            }
          }
          edu.wpi.first.hal.HAL.sendError(isError, code, false, error, locString, traceString.toString(), true);
        }
      }
      */
      ctClassDriverStation.toClass();
    } catch (IOException | NotFoundException | CannotCompileException exception) {
      exception.printStackTrace(AnsiConsole.sysErr());
    }
  }

  /**
   * This class is a StreamHandler that uses ANSI escape codes to colorize the log messages
   */
  public static class LoggingAnsiConsoleHandler extends StreamHandler {
    public LoggingAnsiConsoleHandler() {
      super(ANSI_CONSOLE_STREAM, new LoggingAnsiFormatter());
      setLevel(LEVEL);
    }
    public LoggingAnsiConsoleHandler(OutputStream out) {
      super(out, new LoggingAnsiFormatter());
      setLevel(LEVEL);
    }

    private static class LoggingAnsiFormatter extends Formatter {
      private static final ZoneId ZONE_ID = ZoneId.systemDefault();
      // Specify colors for the different message levels.
      private static final Map<Integer, String> LEVEL_COLORS = Map.of(Level.OFF.intValue(), "", Level.SEVERE.intValue(), ansi().fgBright(Color.RED).toString(), Level.WARNING.intValue(), ansi().fgBright(Color.YELLOW).toString(), Level.INFO.intValue(), ansi().fg(Color.GREEN).toString(), Level.CONFIG.intValue(), ansi().fgBright(Color.BLUE).toString(), Level.FINE.intValue(), ansi().fg(Color.CYAN).toString(), Level.FINER.intValue(), ansi().fg(Color.MAGENTA).toString(), Level.FINEST.intValue(), ansi().fgBright(Color.BLACK).toString(), Level.ALL.intValue(), ansi().fg(Color.DEFAULT).toString());
      private static final String FORMAT = ansi().a("%s").bold().a(Attribute.UNDERLINE).a("[%tb %<td %<tk:%<tM:%<tS.%<tL] %s %s:").boldOff().a(Attribute.UNDERLINE_OFF).a("%s%s").a(Attribute.INTENSITY_FAINT).a("%s").boldOff().reset().newline().toString();
      private static final String RESET = ansi().reset().toString();

      private static String makeStackTraceString(Throwable throwable) {
        StringWriter stringWriter = new StringWriter();
        try (PrintWriter printWriter = new PrintWriter(stringWriter)) {
          printWriter.println();
          throwable.printStackTrace(printWriter);
        }
        StringBuffer stringBuffer = stringWriter.getBuffer();
        return stringBuffer.substring(0, Math.max(0, stringBuffer.length() - 1));
      }

      @Override
      public String format(LogRecord logRecord) {
        ZonedDateTime time = ZonedDateTime.ofInstant(logRecord.getInstant(), ZONE_ID);
        // Get the logger name, source class name, and/or source method name.
        String source = Optional.ofNullable(logRecord.getLoggerName()).or(() -> Optional.ofNullable(logRecord.getSourceClassName())).map(s -> s + " ").orElse("") + Optional.ofNullable(logRecord.getSourceMethodName()).orElse("");
        String message = formatMessage(logRecord);
        // Get the stack trace of the exception if it was thrown.
        String throwable = Optional.ofNullable(logRecord.getThrown()).map(LoggingAnsiFormatter::makeStackTraceString).orElse("");
        // Select the appropriate format string for the log level.
        String color = LEVEL_COLORS.getOrDefault(logRecord.getLevel().intValue(), LEVEL_COLORS.get(Level.ALL.intValue()));

        boolean multiline = message.lines().skip(1).findAny().isPresent();
        boolean ansi = message.contains("\033");
        String prefix = (ansi ? RESET : "") + (multiline ? System.lineSeparator() : " ");

        // Format the log message.
        return String.format(FORMAT, color, time, source, logRecord.getLevel().getLocalizedName(), prefix, message, throwable);
      }

      @Override
      public String getHead(Handler h) {
        return String.format("%s%s level set to %s%s%n", LEVEL_COLORS.get(h.getLevel().intValue()), h.getClass().getSimpleName(), h.getLevel().getName(), RESET);
      }
    }

    @Override
    public synchronized void publish(LogRecord logRecord) {
      super.publish(logRecord);
      flush();
    }
  }

  private static PrintStream printStreamLogger(boolean strip, Consumer<String> logger) {
    return new PrintStream(new ByteArrayOutputStream() {
      @Override
      public void flush() throws IOException {
        String s = new String(buf, 0, strip ? Math.max(0, count - 1) : count);
        if (!s.isBlank()) logger.accept(s);
        reset();
      }
    }, true);
  }
}
