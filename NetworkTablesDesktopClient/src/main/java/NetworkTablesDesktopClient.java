import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Scanner;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTablesDesktopClient {
  public static void main(String[] args) {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    instance.getTable("Recording").addEntryListener((table, key, entry, value, flags) -> {
      Path path = Path.of("..", "src", "main", "deploy", "pathplanner", key);
      File file = path.toFile();
      try {
        System.err.println("Creating path " + key);
        file.createNewFile();
        Files.writeString(file.toPath(), value.getString());
        System.err.println("Recorded path to " + file.getPath());
      } catch (IOException exception) {
        exception.printStackTrace();
      }
    }, EntryListenerFlags.kNew);
    // instance.startClientTeam(4388);
    instance.startClient("localhost");
    instance.startDSClient();
    try (Scanner scanner = new Scanner(System.in)) {
      System.err.println("Press enter to stop...");
      scanner.nextLine();
    }
  }
}
