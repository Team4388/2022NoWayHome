package frc4388.utility;

import static org.fusesource.jansi.Ansi.ansi;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.Map;
import java.util.Optional;
import java.util.logging.ConsoleHandler;
import java.util.logging.Formatter;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import org.fusesource.jansi.Ansi;
import org.fusesource.jansi.Ansi.Attribute;
import org.fusesource.jansi.Ansi.Color;
import org.fusesource.jansi.AnsiConsole;

public class AnsiLogging extends ConsoleHandler {
  public static void systemInstall() {
    try {
      // Configure java.util.logging.Logger to output additional colored information.
      LogManager.getLogManager().updateConfiguration(key -> (o, n) -> {
        switch (key) {
          case ".level":
            return Level.ALL.getName();
          case "handlers":
            return AnsiColorConsoleHandler.class.getName();
          default:
            return n;
        }
      });
      // Replace standard output streams with org.fusesource.jansi.AnsiPrintStreams.
      AnsiConsole.systemInstall();
      // Replace standard output stream with java.util.logging.Logger.
      System.setOut(printStreamLogger(Logger.getGlobal(), Level.INFO));
      // Replace standard error output stream with java.util.logging.Logger.
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
      private final ZoneId zoneId = ZoneId.systemDefault();
      // Specify and prepare formats for messages
      private final Map<Integer, String> levelColors = Map.of(
        Level.OFF.intValue(), "", 
        Level.SEVERE.intValue(), makeMessageFormatString(ansi().fgBright(Color.RED)), 
        Level.WARNING.intValue(), makeMessageFormatString(ansi().fgBright(Color.YELLOW)), 
        Level.INFO.intValue(), makeMessageFormatString(ansi().fg(Color.GREEN)), 
        Level.CONFIG.intValue(), makeMessageFormatString(ansi().fgBright(Color.BLUE)), 
        Level.FINE.intValue(), makeMessageFormatString(ansi().fg(Color.CYAN)), 
        Level.FINER.intValue(), makeMessageFormatString(ansi().fg(Color.MAGENTA)), 
        Level.FINEST.intValue(), makeMessageFormatString(ansi().fgBright(Color.BLACK)), 
        Level.ALL.intValue(), makeMessageFormatString(ansi().fg(Color.DEFAULT))
      );

      private String makeMessageFormatString(Ansi base) {
        return base.bold().a(Attribute.UNDERLINE).a("[%1$tb %1$td %1$tk:%1$tM:%1$tS.%1$tL] %2$s %3$s:").boldOff().a(Attribute.UNDERLINE_OFF).a("%4$s%5$s").a(Attribute.INTENSITY_FAINT).a("%6$s").reset().a("%n").toString();
      }

      private String makeStackTraceString(Throwable throwable) {
        StringWriter stringWriter = new StringWriter();
        try (PrintWriter printWriter = new PrintWriter(stringWriter)) {
          printWriter.println();
          throwable.printStackTrace(printWriter);
        }
        return stringWriter.toString();
      }

      @Override
      public String format(LogRecord logRecord) {
        ZonedDateTime time = ZonedDateTime.ofInstant(logRecord.getInstant(), zoneId);
        String source = Optional.ofNullable(logRecord.getLoggerName()).or(() -> Optional.ofNullable(logRecord.getSourceClassName())).map(s -> s + " ").orElse("") + Optional.ofNullable(logRecord.getSourceMethodName()).orElse("");
        String message = formatMessage(logRecord);
        String throwable = Optional.ofNullable(logRecord.getThrown()).map(this::makeStackTraceString).orElse("");
        String format = levelColors.getOrDefault(logRecord.getLevel().intValue(), levelColors.get(Level.ALL.intValue()));
        return String.format(format, time, source, logRecord.getLevel().getLocalizedName(), message.lines().count() > 1 ? System.lineSeparator() : " ", message, throwable);
      }
    };
  }

  private static PrintStream printStreamLogger(Logger logger, Level level) {
    return new PrintStream(new OutputStream() {
      private final StringBuilder stringBuilder = new StringBuilder();

      @Override
      public void write(int i) throws IOException {
        if (i == '\n') {
          logger.log(level, stringBuilder::toString);
          stringBuilder.setLength(0);
        } else
          stringBuilder.appendCodePoint(i);
      }
    });
  }
}