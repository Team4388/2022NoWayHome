package frc4388.utility.tuner.tunercandevices;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TCAN_TalonFX extends TunerCANDevice {
    private final WPI_TalonFX motor;

    public TCAN_TalonFX(int id) {
        super(id, "FalconFX");
        this.motor = new WPI_TalonFX(this.id);
        this.motor.configFactoryDefault();
    }

    @Override
    void setPosition(double pos) {
        this.motor.set(ControlMode.Position, pos);
    }

    @Override
    void setVelocity(double vel) {
        this.motor.set(ControlMode.Velocity, vel);
    }

    @Override
    void setOutput(double output) {
        this.motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    double getPosition() {
        return this.motor.getSelectedSensorPosition();
    }

    @Override
    double getVelocity() {
        return this.motor.getSelectedSensorVelocity();
    }

    @Override
    double getOutput() {
        return this.motor.getMotorOutputPercent();
    }
}
