package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.Gains;
import frc4388.utility.tuner.TunerLogger;
import frc4388.utility.tuner.annotations.Controller;
import frc4388.utility.tuner.annotations.GainsField;
import frc4388.utility.tuner.annotations.Loggable;
import frc4388.utility.tuner.annotations.Reader;
import frc4388.utility.tuner.TunerController;
import frc4388.utility.tuner.TunerTablesHandler;

import java.util.Map;

public class TestMotor extends SubsystemBase {
    private final WPI_TalonSRX m_testMotor;

    @GainsField(id="TestMotor")
    public Gains gains = new Gains(5, 4, 3, 2, 1, 1);

    @Loggable(id="test")
    public double logable = 4;
    @Loggable(id="test2")
    public double second = 5;
//    private final RelativeEncoder m_testEncoder;

    public TestMotor(WPI_TalonSRX testMotor) {
        m_testMotor = testMotor;
        m_testMotor.configFactoryDefault();
//        m_testEncoder = m_testMotor.getEncoder();

        TunerTablesHandler.getInstance().addControllers(TunerController.createTunerControllers(this));
        TunerTablesHandler.getInstance().addLoggers(TunerLogger.createTunerLoggers(this));
    }

//    public void testDesmos() {
//        DesmosServer.putDouble("Position", m_testEncoder.getPosition());
//        m_testMotor.set(DesmosServer.readDouble("Speed"));
//    }

    @Override
    public void periodic() {
	logable = Math.random()*2 -1;
    }

    @Reader(id="TestMotor", value="velocity")
    public double velocityReader() {
        return logable;
    }

    @Reader(id="TestMotor", value="position")
    public double positionReader() {
        return Math.random() * 2 - 1;
    }

    @Controller(id="TestMotor", value="velocity")
    public void velocityController(double value) {
        logable = Math.random() * 10;
        m_testMotor.set(value);
    }
}
