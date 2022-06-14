package frc4388.utility.tuner;

import edu.wpi.first.hal.can.CANJNI;
import frc4388.utility.tuner.tunercandevices.TCAN_SparkMax;
import frc4388.utility.tuner.tunercandevices.TCAN_TalonFX;
import frc4388.utility.tuner.tunercandevices.TCAN_TalonSRX;
import frc4388.utility.tuner.tunercandevices.TunerCANDevice;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

// WARN This class may not be possible to test until I have access to actual hardware
public class CANBusReader {
    private static final class DeviceSignatures {
        // CTRE
        static final int PDP = 0x08041400; static final int PCM = 0x09041400; static final int TalonSRX = 0x02041400;
        static final int TalonFX = -1;
        // REV robotics
        static final int SparkMax = -1;

        // ? Should this be a method
        static final int[] asArray = new int[] {PDP, PCM, TalonSRX, TalonFX, SparkMax};

        private DeviceSignatures() {}
    }

    // * Basically copied from https://www.chiefdelphi.com/t/how-to-detect-missing-can-devices-from-java/147675/10 with
    // * a few small changes
    private static long checkMessage(int fullId, int deviceID) {
        try {
            ByteBuffer targetID = ByteBuffer.allocateDirect(4).order(ByteOrder.LITTLE_ENDIAN);
            ByteBuffer timeStamp = ByteBuffer.allocateDirect(4).order(ByteOrder.LITTLE_ENDIAN);

            targetID.asIntBuffer().put(0,fullId | deviceID);
            timeStamp.asIntBuffer().put(0,0x00000000);

            // Literally magic.
            // ? this may not work in sim.
            CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetID.asIntBuffer(), 0x1fffffff, timeStamp);

            long retval = timeStamp.getInt();
            retval &= 0xFFFFFFFF; /* undo sign-extension */
            return retval;
        } catch (Exception e) {
            return -1;
        }
    }

    // TODO: Make this method work. May need actual hardware for testing
    // https://www.chiefdelphi.com/t/how-to-detect-missing-can-devices-from-java/147675/10
    // ^ Really f#cking useful
    // This is basically copied from the link above except rewritten because I didn't like how the official ctre person did it
    public static List<TunerCANDevice> readCANBus() {
        List<TunerCANDevice> activeDevices = new ArrayList<>();

        long pdpTimestamp = checkMessage(DeviceSignatures.PDP, 0);

        // ! Rather poorly done data structure: ArrayList<Pair<Integer, Long>>[63]
        // ? But maybe the best option

        // Lots of arrays because I don't want to make a proper class that holds multiple Pair values
        long[] srxTimestamps = new long[63];
        long[] fxTimestamps = new long[63];
        long[] smaxTimestamps = new long[63];

        for(int i = 0; i < 63; i++) {
            srxTimestamps[i] = checkMessage(DeviceSignatures.TalonSRX, i+1);
            fxTimestamps[i] = checkMessage(DeviceSignatures.TalonFX, i+1);
            smaxTimestamps[i] = checkMessage(DeviceSignatures.SparkMax, i+1);
        }

        // Will cause a loop overrun
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // Writing my own class would reduce most of this copy/paste
        for(int i = 0; i < 63; i++) {
            long srxTimestamp = checkMessage(DeviceSignatures.TalonSRX, i+1);
            long fxTimestamp = checkMessage(DeviceSignatures.TalonFX, i+1);
            long smaxTimestamp = checkMessage(DeviceSignatures.SparkMax, i+1);

            if(srxTimestamps[i] >= 0 && srxTimestamp >= 0 && srxTimestamps[i] != srxTimestamp)
                activeDevices.add(new TCAN_TalonSRX(i));
            else if(fxTimestamps[i] >= 0 && fxTimestamp >= 0 && fxTimestamps[i] != fxTimestamp)
                activeDevices.add(new TCAN_TalonFX(i));
            else if(smaxTimestamps[i] >= 0 && smaxTimestamp >= 0 && smaxTimestamps[i] != smaxTimestamp)
                activeDevices.add(new TCAN_SparkMax(i));
        }

        return activeDevices;
    }
}
