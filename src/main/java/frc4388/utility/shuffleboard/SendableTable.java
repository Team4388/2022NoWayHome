package frc4388.utility.shuffleboard;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc4388.robot.subsystems.BoomBoom.ShooterTableEntry;

public class SendableTable implements Sendable {
  private Supplier<ShooterTableEntry[]> m_table;
  public SendableTable(Supplier<ShooterTableEntry[]> table) {
    m_table = table;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Table");
    builder.addRawProperty("table", this::getTableAsBytes, null);
    builder.addStringArrayProperty("header", () -> new String[] {"distance", "hoodExt", "drumVelocity"}, null);
  }
  private byte[] getTableAsBytes() {
    ShooterTableEntry[] table = m_table.get();
    ByteBuffer byteBuffer = ByteBuffer.allocate(Double.BYTES * 3 * table.length);
    Arrays.stream(table).forEach(e -> {
      byteBuffer.putDouble(e.distance);
      byteBuffer.putDouble(e.hoodExt);
      byteBuffer.putDouble(e.drumVelocity);
    });
    return byteBuffer.hasArray() ? byteBuffer.array() : new byte[0];
  }
  private void setTableFromBytes(byte[] bytes) {
    ByteBuffer byteBuffer = ByteBuffer.wrap(bytes);
    ShooterTableEntry[] table = m_table.get();
    for (int i = 0; i < table.length; i++) {
      table[i].distance = byteBuffer.getDouble();
      table[i].hoodExt = byteBuffer.getDouble();
      table[i].drumVelocity = byteBuffer.getDouble();
    }
  }
}
