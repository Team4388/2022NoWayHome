package frc4388.utility.tuner.tunercandevices;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TCAN_TalonSRX extends TunerCANDevice {
    private final WPI_TalonSRX motor;

    public TCAN_TalonSRX(int id) {
        super(id, "FalconSRX");
        motor = new WPI_TalonSRX(this.id);
        motor.configFactoryDefault();
    }

    @Override
    void setPosition(double pos) {
        motor.set(ControlMode.Position, pos);
    }

    @Override
    void setVelocity(double vel) {
        motor.set(ControlMode.Velocity, vel);
    }

    @Override
    void setOutput(double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    double getPosition() {
        return motor.getSelectedSensorPosition();
    }

    @Override
    double getVelocity() {
        return motor.getSelectedSensorVelocity();
    }

    @Override
    double getOutput() {
        return motor.getMotorOutputPercent();
    }
}
