package frc4388.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.DesmosServer;

public class TestMotor extends SubsystemBase {
    private CANSparkMax m_testMotor;
    private RelativeEncoder m_testEncoder;

    public TestMotor(CANSparkMax testMotor) {
        m_testMotor = testMotor;
        m_testEncoder = m_testMotor.getEncoder();
    }

    public void testDesmos() {
        DesmosServer.putDouble("Position", m_testEncoder.getPosition());
        // m_testMotor.set(DesmosServer.readDouble("Speed"));
    }
}
