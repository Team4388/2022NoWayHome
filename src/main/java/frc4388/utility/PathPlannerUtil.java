package frc4388.utility;

import java.io.File;
import java.io.Writer;
import java.util.Optional;

import com.diffplug.common.base.Errors;
import com.fasterxml.jackson.annotation.JsonInclude.Include;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.datatype.jdk8.Jdk8Module;

import edu.wpi.first.math.geometry.Translation2d;

public final class PathPlannerUtil {
  public static final class Path {
    public Optional<Waypoint[]> waypoints;
    public Optional<Double> maxVelocity;
    public Optional<Double> maxAcceleration;
    public Optional<Boolean> isReversed;

    private static final ObjectMapper objectMapper = new ObjectMapper();
    static {
      objectMapper.registerModule(new Jdk8Module());
      objectMapper.setSerializationInclusion(Include.ALWAYS);
    }

    public static Path read(File src) {
      return Errors.log().getWithDefault(() -> objectMapper.readValue(src, Path.class), null);
    }

    public void write(File resultFile) {
      Errors.log().run(() -> objectMapper.writeValue(resultFile, this));
    }

    public void write(Writer writer) {
      Errors.log().run(() -> objectMapper.writeValue(writer, this));
    }

    public static final class Waypoint {
      public Optional<Translation2d> anchorPoint;
      public Optional<Translation2d> prevControl;
      public Optional<Translation2d> nextControl;
      public Optional<Double> holonomicAngle;
      public Optional<Boolean> isReversal;
      public Optional<Double> velOverride;
      public Optional<Boolean> isLocked;

      public Waypoint() {
      }

      public Waypoint(Translation2d anchorPoint, Translation2d prevControl, Translation2d nextControl, Double holonomicAngle, Boolean isReversal, Double velOverride, Boolean isLocked) {
        this.anchorPoint = Optional.ofNullable(anchorPoint);
        this.prevControl = Optional.ofNullable(prevControl);
        this.nextControl = Optional.ofNullable(nextControl);
        this.holonomicAngle = Optional.ofNullable(holonomicAngle);
        this.isReversal = Optional.ofNullable(isReversal);
        this.velOverride = Optional.ofNullable(velOverride);
        this.isLocked = Optional.ofNullable(isLocked);
      }
    }
  }
}
