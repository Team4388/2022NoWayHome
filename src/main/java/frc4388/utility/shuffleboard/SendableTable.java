package frc4388.utility.shuffleboard;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc4388.robot.subsystems.BoomBoom.ShooterTableEntry;

public class SendableTable implements Sendable {
  private Supplier<ShooterTableEntry[]> m_tableGetter;
  private Consumer<ShooterTableEntry[]> m_tableSetter;
  private ShooterTableEntry[] tableCache;
  private byte[] bytesCache;

  public SendableTable(Supplier<ShooterTableEntry[]> getter, Consumer<ShooterTableEntry[]> setter) {
    m_tableGetter = getter;
    m_tableSetter = setter;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Table");
    builder.addRawProperty("table", this::getTableAsBytes, this::setTableFromBytes);
    builder.addStringArrayProperty("header", () -> new String[] { "distance", "hoodExt", "drumVelocity" }, null);
  }

  private byte[] getTableAsBytes() {
    ShooterTableEntry[] table = m_tableGetter.get();
    if (!Arrays.equals(tableCache, table)) {
      ByteBuffer byteBuffer = ByteBuffer.allocate(Double.BYTES * 3 * table.length);
      Arrays.stream(table).forEach(e -> {
        byteBuffer.putDouble(e.distance);
        byteBuffer.putDouble(e.hoodExt);
        byteBuffer.putDouble(e.drumVelocity);
      });
      tableCache = table;
      bytesCache = byteBuffer.array();
    }
    return bytesCache;
  }

  private void setTableFromBytes(byte[] bytes) {
    if (bytes.length > 0 && !Arrays.equals(bytesCache, bytes)) {
      ByteBuffer byteBuffer = ByteBuffer.wrap(bytes);
      ShooterTableEntry[] table = new ShooterTableEntry[bytes.length / (3 * Double.BYTES)];
      for (int i = 0; i < table.length; i++) {
        table[i].distance = byteBuffer.getDouble();
        table[i].hoodExt = byteBuffer.getDouble();
        table[i].drumVelocity = byteBuffer.getDouble();
      }
      m_tableSetter.accept(table);
    }
  }
}
