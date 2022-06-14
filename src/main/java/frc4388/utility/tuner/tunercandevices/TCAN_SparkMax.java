package frc4388.utility.tuner.tunercandevices;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class TCAN_SparkMax extends TunerCANDevice {
    private final CANSparkMax motor;

    public TCAN_SparkMax(int id) {
        super(id, "SparkMax");
        this.motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    @Override
    void setPosition(double pos) {
        this.motor.getPIDController().setReference(pos, CANSparkMax.ControlType.kPosition);
    }

    @Override
    void setVelocity(double vel) {
        this.motor.getPIDController().setReference(vel, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    void setOutput(double output) {
        this.motor.set(output);
    }

    @Override
    double getPosition() {
        return this.motor.getEncoder().getPosition();
    }

    @Override
    double getVelocity() {
        return this.motor.getEncoder().getVelocity();
    }

    @Override
    double getOutput() {
        return this.motor.get();
    }
}
