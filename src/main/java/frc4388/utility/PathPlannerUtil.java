package frc4388.utility;

import java.io.File;
import java.util.Optional;
import com.diffplug.common.base.Errors;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.datatype.jdk8.Jdk8Module;

public class PathPlannerUtil {
  public static final class Path {
    public Optional<Waypoint>[] waypoints;

    public static final class Waypoint {
      public Optional<Point> anchorPoint;
      public Optional<Point> prevControl;
      public Optional<Point> nextControl;

      public static final class Point {
        public Optional<Double> x;
        public Optional<Double> y;
      }

      public Optional<Double> holonomicAngle;
      public Optional<Boolean> isReversal;
      public Optional<Boolean> velOverride;
      public Optional<Boolean> isLocked;
    }

    public Optional<Double> maxVelocity;
    public Optional<Double> maxAcceleration;
    public Optional<Boolean> isReversed;

    private static final ObjectMapper objectMapper = new ObjectMapper();
    static { objectMapper.registerModule(new Jdk8Module()); }

    public static Path read(File src) {
      return Errors.log().getWithDefault(() -> objectMapper.readValue(src, Path.class), null);
    }

    public void write(File resultFile) {
      Errors.log().run(() -> objectMapper.writeValue(resultFile, this));
    }

    @Override
    public String toString() {
      return Errors.log().getWithDefault(() -> objectMapper.writeValueAsString(this), super.toString());
    }
  }
}
