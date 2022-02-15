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
      public Waypoint() {}
      public Waypoint(Double anchorPointX, Double anchorPointY, Double prevControlX, Double prevControlY, Double nextControlX, Double nextControlY, Double holonomicAngle, Boolean isReversal, Double velOverride, Boolean isLocked) {
        this.anchorPoint = anchorPointX == null && anchorPointY == null ? Optional.empty() : Optional.of(Point.of(anchorPointX, anchorPointY));
        this.prevControl = prevControlX == null && prevControlY == null ? Optional.empty() : Optional.of(Point.of(prevControlX, prevControlY));
        this.nextControl = nextControlX == null && nextControlY == null ? Optional.empty() : Optional.of(Point.of(nextControlX, nextControlY));
        this.holonomicAngle = Optional.ofNullable(holonomicAngle);
        this.isReversal = Optional.ofNullable(isReversal);
        this.velOverride = Optional.ofNullable(velOverride);
        this.isLocked = Optional.ofNullable(isLocked);
      }
      public Optional<Point> anchorPoint;
      public Optional<Point> prevControl;
      public Optional<Point> nextControl;

      public static final class Point {
        public Point() {}
        public static Point of(Double x, Double y) {
          Point point = new Point();
          point.x = Optional.ofNullable(x);
          point.y = Optional.ofNullable(y);
          return point;
        }
        public Optional<Double> x;
        public Optional<Double> y;
      }

      public Optional<Double> holonomicAngle;
      public Optional<Boolean> isReversal;
      public Optional<Double> velOverride;
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
