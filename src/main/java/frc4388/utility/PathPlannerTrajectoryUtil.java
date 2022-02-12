package frc4388.utility;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.logging.Logger;
import java.util.stream.Collectors;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class PathPlannerTrajectoryUtil {

  // public static final class BetterUtil {

  //   public static interface IPlanet {
  //     String getName();
  //   }

  //   static class Planet implements IPlanet {
  //     private String name;

  //     public String getName() {
  //       return name;
  //     }

  //     public void setName(String iName) {
  //       name = iName;
  //     }
  //   }

  //   public static void test() throws JsonProcessingException {
  //     IPlanet ip = getProxy(IPlanet.class, new Planet());
  //     ObjectMapper mapper = new ObjectMapper();
  //     mapper.writeValueAsString(ip);
  //   }

  //   public static <T> T getProxy(Class<T> type, Object obj) {
  //     class ProxyUtil implements InvocationHandler {
  //       Object obj;

  //       public ProxyUtil(Object o) {
  //         obj = o;
  //       }

  //       @Override
  //       public Object invoke(Object proxy, Method m, Object[] args) throws Throwable {
  //         Object result = null;
  //         result = m.invoke(obj, args);
  //         return result;
  //       }
  //     }
  //     @SuppressWarnings("unchecked")
  //     T proxy = (T) Proxy.newProxyInstance(type.getClassLoader(), new Class[] { type }, new ProxyUtil(obj));
  //     return proxy;
  //   }
  // }

  public static PathPlannerTrajectory fromPathPlannerJson(Path path, double maxVel, double maxAccel, boolean reversed)
      throws IOException {
    if (path.endsWith(".path"))
      return PathPlanner.loadPath(path.toFile().getName(), maxVel, maxAccel, reversed);
    throw new IOException("Invalid path");
  }

  public static PathPlannerTrajectory fromPathPlannerJson(Path path, double maxVel, double maxAccel)
      throws IOException {
    return fromPathPlannerJson(path, maxVel, maxAccel, false);
  }

  @SuppressWarnings("unchecked")
  public static PathPlannerTrajectory fromPathweaverJson(Path path, double maxVel, double maxAccel) throws IOException {
    Object obj = JSONValue.parse(Files.newBufferedReader(path));
    JSONArray array = (JSONArray) obj;
    List<PathPlannerState> states = (List<PathPlannerState>) array.stream().map(o -> {
      JSONObject jo = (JSONObject) o;
      PathPlannerState pathPlannerState = new PathPlannerState();
      pathPlannerState.timeSeconds = Double.parseDouble(Objects.toString(jo.get("time")));
      pathPlannerState.velocityMetersPerSecond = Double.parseDouble(Objects.toString(jo.get("velocity")));
      pathPlannerState.accelerationMetersPerSecondSq = Double.parseDouble(Objects.toString(jo.get("acceleration")));
      JSONObject pose = (JSONObject) jo.get("pose");
      JSONObject rotation = (JSONObject) pose.get("rotation");
      JSONObject translation = (JSONObject) pose.get("translation");
      double radians = Double.parseDouble(Objects.toString(rotation.get("radians")));
      double x = Double.parseDouble(Objects.toString(translation.get("x")));
      double y = Double.parseDouble(Objects.toString(translation.get("y")));
      pathPlannerState.poseMeters = new Pose2d(x, y, new Rotation2d(radians));
      pathPlannerState.curvatureRadPerMeter = Double.parseDouble(Objects.toString(jo.get("curvature")));
      pathPlannerState.positionMeters = Double.parseDouble(Objects.toString(jo.get("position")));
      pathPlannerState.angularVelocity = new Rotation2d(
          Double.parseDouble(Objects.toString(jo.get("angularVelocity"))));
      pathPlannerState.angularAcceleration = new Rotation2d(
          Double.parseDouble(Objects.toString(jo.get("angularAcceleration"))));
      pathPlannerState.holonomicRotation = new Rotation2d(
          Double.parseDouble(Objects.toString(jo.get("holonomicRotation"))));
      Logger.getAnonymousLogger().finest(pathPlannerState.toString());
      return pathPlannerState;
    }).collect(Collectors.toList());
    try {
      var constructor = PathPlannerTrajectory.class.getDeclaredConstructor(List.class);
      constructor.setAccessible(true);
      var a = constructor.newInstance(states);
      Logger.getAnonymousLogger()
          .severe(() -> a.getStates().stream().map(PathPlannerState.class::cast).map(o -> String.format(
              "State(Sec: %.2f, Vel m/s: %.2f, Accel m/s/s: %.2f, Pose: %s, Curvature: %.2f, positionMeters: %.2f, angularVelocity: %s, angularAcceleration: %s, holonomicRotation: %s)",
              o.timeSeconds,
              o.velocityMetersPerSecond,
              o.accelerationMetersPerSecondSq,
              o.poseMeters,
              o.curvatureRadPerMeter,
              o.positionMeters,
              o.angularVelocity,
              o.angularAcceleration,
              o.holonomicRotation)).collect(Collectors.joining("\n")));
      return a;
    } catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException
        | NoSuchMethodException | SecurityException e) {
      throw new RuntimeException(e);
    }
  }

  public static void toPathweaverJson(Collection<PathPlannerState> trajectoryStates, Path path) throws IOException {
    try (BufferedWriter out = new BufferedWriter(new FileWriter(path.toFile()))) {
      out.write('[');
      for (var state : trajectoryStates) {
        double acceleration = state.accelerationMetersPerSecondSq;
        double curvature = state.curvatureRadPerMeter;
        // pose
        // rotation
        double poseRotationRadians = state.poseMeters.getRotation().getRadians();
        // translation
        double poseTranslationX = state.poseMeters.getX();
        double poseTranslationY = state.poseMeters.getY();
        double time = 0.0;
        double velocity = state.velocityMetersPerSecond;
        double position = state.positionMeters;
        double holonomicRotation = state.holonomicRotation.getRadians();
        double angularVelocity = state.angularVelocity.getRadians();
        double angularAcceleration = state.angularAcceleration.getRadians();
        double curveRadius = 0;
        out.write('{');
        out.write("\"acceleration\":" + acceleration + ",");
        out.write("\"curvature\":" + curvature + ",");
        out.write("\"pose\": {");
        out.write("\"rotation\": {");
        out.write("\"radians\":" + poseRotationRadians);
        out.write("},");
        out.write("\"translation\": {");
        out.write("\"x\":" + poseTranslationX + ",");
        out.write("\"y\":" + poseTranslationY);
        out.write('}');
        out.write("},");
        out.write("\"time\":" + time + ",");
        out.write("\"velocity\":" + velocity + ",");
        out.write("\"position\":" + position + ",");
        out.write("\"holonomicRotation\":" + holonomicRotation + ",");
        out.write("\"angularVelocity\":" + angularVelocity + ",");
        out.write("\"angularAcceleration\":" + angularAcceleration + ",");
        out.write("\"curveRadius\":" + curveRadius);
        out.write("},");
      }
      out.write(']');
      out.flush();
    }
  }
}
