package frc4388.utility.tuner;

import frc4388.utility.tuner.tunercandevices.TunerCANDevice;

/* ! -------------------------------------------------------------------------------------------------------------------
   ! ------------------------------------- THIS CLASS IS POTENTIALLY UNNECESSARY: --------------------------------------
   ! ---------------         Having a factory class that is independent of the CANBusReader class      -----------------
   ! ---------------    is probably too much boilerplate and too much room for something to go wrong.  -----------------
   ! ---------------    On the other hand a factory that can take device ids and produce the correct   -----------------
   ! ---------------      device controller will make adding autodetected types easier to implement.   -----------------
   ! ------------------------------------------------ Think more later -------------------------------------------------
   ! -----------------------------------------------------------------------------------------------------------------*/
public class TunerCANDeviceFactory {
    public static TunerCANDevice getCANDevice(byte i) {
        // ? :( Why no switch expressions ?
        //// return switch(i) {
        ////      case 0 -> null;
        ////      case 1 -> null;
        ////      default -> null;
        //// }

        switch (i) {
            case 0: return null;
            case 1: return null;
            default: return null;
        }
    }
}
