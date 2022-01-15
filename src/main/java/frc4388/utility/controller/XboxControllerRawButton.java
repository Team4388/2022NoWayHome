package frc4388.utility.controller;

import edu.wpi.first.wpilibj2.command.button.Button;

public class XboxControllerRawButton extends Button {
    private final XboxControllerRaw m_controller;
    private final int m_buttonNumber;

    public XboxControllerRawButton(XboxControllerRaw controller, int buttonNumber) {
        m_controller = controller;
        m_buttonNumber = buttonNumber;
    }

    @Override
    public boolean get() {
        return m_controller.getButton(m_buttonNumber);
    }
}
