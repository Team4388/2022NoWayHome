package frc4388.utility;

import static org.fusesource.jansi.Ansi.ansi;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.logging.ConsoleHandler;
import java.util.logging.Formatter;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import org.fusesource.jansi.Ansi;
import org.fusesource.jansi.Ansi.Attribute;
import org.fusesource.jansi.Ansi.Color;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.fusesource.jansi.AnsiConsole;

public class AnsiLogging extends ConsoleHandler {
  public static void systemInstall() {
    try {
      // Configures java.util.logging.Logger to output additional colored information.
      LogManager.getLogManager().updateConfiguration(key -> (o, n) -> {
        switch (key) {
          case ".level": return Level.ALL.getName();
          case "handlers": return AnsiColorConsoleHandler.class.getName();
          default: return n;
        }
      });
      // Replaces standard output streams with org.fusesource.jansi.AnsiPrintStreams.
      AnsiConsole.systemInstall();
      // Replaces standard output stream with java.util.logging.Logger.
      System.setOut(printStreamLogger(Logger.getGlobal(), Level.ALL));
      // Replaces standard error output stream with java.util.logging.Logger.
      System.setErr(printStreamLogger(Logger.getGlobal(), Level.SEVERE));
    } catch (IOException exception) {
      exception.printStackTrace(AnsiConsole.sysErr());
    }
  }
  
  public static class AnsiColorConsoleHandler extends ConsoleHandler {
    @Override
    public void publish(LogRecord logRecord) {
      AnsiConsole.err().print(getFormatter().format(logRecord));
      AnsiConsole.err().flush();
    }
    @Override
    public Formatter getFormatter() {
      return formatter;
    }
    private static final Formatter formatter = new Formatter() {
      @Override
      public String format(LogRecord logRecord) {
        ZonedDateTime zdt = ZonedDateTime.ofInstant(logRecord.getInstant(), ZoneId.systemDefault());
        String source;
        if (logRecord.getSourceClassName() != null && !logRecord.getSourceClassName().startsWith(AnsiLogging.class.getName())) {
          source = logRecord.getSourceClassName();
          if (logRecord.getSourceMethodName() != null) {
            source += " " + logRecord.getSourceMethodName();
          }
        } else
          source = logRecord.getLoggerName();
        String message = formatMessage(logRecord);
        boolean multiline = message.contains("\n");
        String throwable = "";
        if (logRecord.getThrown() != null) {
          StringWriter sw = new StringWriter();
          try (PrintWriter pw = new PrintWriter(sw)) {
            pw.println();
            logRecord.getThrown().printStackTrace(pw);
          }
          throwable = sw.toString();
          multiline = true;
        }
        Ansi ansi;
        if (logRecord.getLevel() == Level.SEVERE) ansi = ansi().fgBright(Color.RED);
        else if (logRecord.getLevel() == Level.WARNING) ansi = ansi().fgBright(Color.YELLOW);
        else if (logRecord.getLevel() == Level.INFO) ansi = ansi().fg(Color.GREEN);
        else if (logRecord.getLevel() == Level.CONFIG) ansi = ansi().fgBright(Color.BLUE);
        else if (logRecord.getLevel() == Level.FINE) ansi = ansi().fg(Color.CYAN);
        else if (logRecord.getLevel() == Level.FINER) ansi = ansi().fg(Color.MAGENTA);
        else if (logRecord.getLevel() == Level.FINEST) ansi = ansi().fgBright(Color.BLACK);
        else ansi = ansi().fg(Color.DEFAULT);
        String format = ansi.bold().a(Attribute.UNDERLINE).a("%1$tb %1$td, %1$tY %1$tl:%1$tM:%1$tS %1$Tp %2$s %4$s:").boldOff().a(Attribute.UNDERLINE_OFF).a(multiline ? "%n%5$s%6$s" : " %5$s%6$s").reset().a("%n").toString();
        return String.format(format, zdt, source, logRecord.getLoggerName(), logRecord.getLevel().getLocalizedName(), message, throwable);
      }
    };
  }

  private static PrintStream printStreamLogger(Logger logger, Level level) {
    return new PrintStream(new OutputStream() {
      private final StringBuilder stringBuilder = new StringBuilder();

      @Override
      public final void write(int i) throws IOException {
        if (i == '\n') {
          logger.log(level, stringBuilder::toString);
          stringBuilder.setLength(0);
        } else
          stringBuilder.appendCodePoint(i);
      }
    });
  }
}