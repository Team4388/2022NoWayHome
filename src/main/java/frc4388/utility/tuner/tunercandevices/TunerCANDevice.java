package frc4388.utility.tuner.tunercandevices;

import frc4388.utility.tuner.annotations.Controller;
import frc4388.utility.tuner.annotations.Reader;
import frc4388.utility.tuner.TunerController;

import java.util.Map;

// ! Could be an interface, but that would be boring
// * serious: the constructor is one bit of boilerplate I would rather not have to write again
public abstract class TunerCANDevice {
    protected final int id;
    protected final String name;

    public TunerCANDevice(int id, String name) {
        this.id = id;
        this.name = name;
    }

    public Map<String, TunerController> getTunerController() {
        Map<String, TunerController> controllers = TunerController.createTunerControllers(this);
        TunerController controller = controllers.remove("TunerCANDevice");
        controllers.put(name + ": " + id, controller);
        return controllers;
    }

    @Controller(id="TunerCANDevice", value="position")
    abstract void setPosition(double pos);
    @Controller(id="TunerCANDevice", value="velocity")
    abstract void setVelocity(double vel);
    @Controller(id="TunerCANDevice", value="output")
    abstract void setOutput(double output);

    @Reader(id="TunerCANDevice", value="position")
    abstract double getPosition();
    @Reader(id="TunerCANDevice", value="velocity")
    abstract double getVelocity();
    @Reader(id="TunerCANDevice", value="output")
    abstract double getOutput();
}
