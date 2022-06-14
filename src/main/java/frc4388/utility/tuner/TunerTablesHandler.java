package frc4388.utility.tuner;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc4388.utility.Gains;
import frc4388.utility.tuner.tunercandevices.TunerCANDevice;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TunerTablesHandler {
    private final NetworkTable tunerTable;
    private final NetworkTableEntry activeEntry;

    // ? Should NetworkTable be the key or the value ?
    private final Map<NetworkTable, TunerController> controllerNetworkTableMap;
    private final Map<NetworkTableEntry, TunerLogger> loggerNetworkEntryMap;
    private boolean hasNotBeenActivated = true;
    private List<TunerCANDevice> canBus; // * Needs to be a field so that the devices are loaded into memory
                                         // * Although, since the object belongs to the TunerController, the garbage collector might not get rid the devices

    private static TunerTablesHandler instance;

    public static TunerTablesHandler getInstance() {
        return instance != null ? instance : (instance = new TunerTablesHandler());
    }

    private TunerTablesHandler() {
        this.tunerTable = NetworkTableInstance.getDefault().getTable("TunerTables");
        this.activeEntry = this.tunerTable.getEntry("active");
        this.activeEntry.setBoolean(false);

        this.controllerNetworkTableMap = new HashMap<>();
        this.loggerNetworkEntryMap = new HashMap<>();
    }

    public void addControllers(Map<String, TunerController> controllerMap) {
        for(String name : controllerMap.keySet()) {
            TunerController controller = controllerMap.get(name);

            NetworkTable subTable = tunerTable.getSubTable(name);
            controllerNetworkTableMap.put(subTable, controller);

            NetworkTable controllerTable = subTable.getSubTable("ControllerTable");
            controllerTable.addEntryListener((table, key, entry, value, flags) -> {
                if(activeEntry.getBoolean(false))
                    controller.setTarget(key, value.getDouble());
            }, EntryListenerFlags.kUpdate);

            for(String controllerName : controller.getNames())
                controllerTable.getEntry(controllerName).setDouble(0);

            NetworkTable gainsTable = subTable.getSubTable("GainsTable");
            gainsTable.addEntryListener((table, key, entry, value, flags) -> {
                if(activeEntry.getBoolean(false))
                    controller.setGains(
                            gainsTable.getEntry("kP").getDouble(0),
                            gainsTable.getEntry("kI").getDouble(0),
                            gainsTable.getEntry("kD").getDouble(0),
                            gainsTable.getEntry("kF").getDouble(0)
                    );
            }, EntryListenerFlags.kUpdate);

            // TODO: make these values reflect default values if gains field annotation is involved
            Gains defaultGains = controller.getGains();

            gainsTable.getEntry("kP").setDouble(defaultGains.m_kP);
            gainsTable.getEntry("kI").setDouble(defaultGains.m_kI);
            gainsTable.getEntry("kD").setDouble(defaultGains.m_kD);
            gainsTable.getEntry("kF").setDouble(defaultGains.m_kF);
        }
    }

    public void addLoggers(Map<String, TunerLogger> loggerMap) {
        NetworkTable loggerTable = tunerTable.getSubTable("LoggerTable");

        for(String name : loggerMap.keySet()) {
            TunerLogger logger = loggerMap.get(name);
            NetworkTableEntry loggerEntry = loggerTable.getEntry(name);

            loggerNetworkEntryMap.put(loggerEntry, logger);
        }
    }

    public void writeReadersToNetTables(NetworkTable table, TunerController controller) {
        for(String readerName : controller.getNames())
            table.getEntry(readerName).setDouble(controller.readValue(readerName));
    }

    public void updateReaders() {
        if(!activeEntry.getBoolean(false)) return;

        if(hasNotBeenActivated) {
            canBus = CANBusReader.readCANBus();
            for (TunerCANDevice device : canBus)
                addControllers(device.getTunerController());

            hasNotBeenActivated = false;
        }

        for(NetworkTable table : controllerNetworkTableMap.keySet()) {
            TunerController controller = controllerNetworkTableMap.get(table);
            writeReadersToNetTables(table.getSubTable("ReaderTable"), controller);
        }

        // TODO: make always active work properly
        for(NetworkTableEntry entry : loggerNetworkEntryMap.keySet()) {
            entry.setDouble(loggerNetworkEntryMap.get(entry).getCurrentValue());
        }
    }
}
