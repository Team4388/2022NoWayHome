package frc4388.utility.controller;

import java.nio.ByteBuffer;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class RawXboxController extends XboxController {
  private final int m_port;

  public RawXboxController(int port) {
    super(port);
    if (port < 0 || port >= DriverStation.kJoystickPorts)
      throw new IllegalArgumentException("Joystick index is out of range, should be 0-5");
    m_port = port;
  }

  private static class HALJoystickButtons {
    public int m_buttons;
    public byte m_count;
  }

  private static class HALJoystickAxes {
    public float[] m_axes;
    public short m_count;

    HALJoystickAxes(int count) {
      m_axes = new float[count];
    }
  }

  private static class HALJoystickPOVs {
    public short[] m_povs;
    public short m_count;

    HALJoystickPOVs(int count) {
      m_povs = new short[count];
      for (int i = 0; i < count; i++) {
        m_povs[i] = -1;
      }
    }
  }

  private static final double JOYSTICK_UNPLUGGED_MESSAGE_INTERVAL = 1.0;
  private static double m_nextMessageTime;

  private HALJoystickAxes m_joystickAxes = new HALJoystickAxes(HAL.kMaxJoystickAxes);
  private HALJoystickPOVs m_joystickPOVs = new HALJoystickPOVs(HAL.kMaxJoystickAxes);
  private HALJoystickButtons m_joystickButtons = new HALJoystickButtons();

  // Joystick button rising/falling edge flags
  private int m_joystickButtonsPressed = 0;
  private int m_joystickButtonsReleased = 0;

  private static final ByteBuffer m_buttonCountBuffer = ByteBuffer.allocateDirect(1);

  @Override
  public double getRawAxis(int axis) {
    return getStickAxis(axis);
  }

  @Override
  public boolean getRawButton(int button) {
    return getStickButton((byte) button);
  }

  @Override
  public boolean getRawButtonPressed(int button) {
    return getStickButtonPressed((byte) button);
  }

  @Override
  public boolean getRawButtonReleased(int button) {
    return getStickButtonReleased(button);
  }

  @Override
  public int getPOV(int pov) {
    return getStickPOV(pov);
  }

  /**
   * The state of one joystick button. Button indexes begin at 1.
   *
   * @param button The button index, beginning at 1.
   * @return The state of the joystick button.
   */
  public boolean getStickButton(final int button) {
    if (button <= 0) {
      reportJoystickUnpluggedError();
      return false;
    }
    if (button <= m_joystickButtons.m_count) {
      return (m_joystickButtons.m_buttons & 1 << (button - 1)) != 0;
    }
    reportJoystickUnpluggedWarning(button, m_port);
    return false;
  }

  /**
   * Whether one joystick button was pressed since the last check. Button indexes begin at 1.
   *
   * @param button The button index, beginning at 1.
   * @return Whether the joystick button was pressed since the last check.
   */
  public boolean getStickButtonPressed(final int button) {
    getData();
    if (button <= 0) {
      reportJoystickUnpluggedError();
      return false;
    }
    if (button <= m_joystickButtons.m_count) {
      // If button was pressed, clear flag and return true
      if ((m_joystickButtonsPressed & 1 << (button - 1)) != 0) {
        m_joystickButtonsPressed &= ~(1 << (button - 1));
        return true;
      } else {
        return false;
      }
    }
    reportJoystickUnpluggedWarning(button, m_port);
    return false;
  }

  /**
   * Whether one joystick button was released since the last check. Button indexes begin at 1.
   *
   * @param button The button index, beginning at 1.
   * @return Whether the joystick button was released since the last check.
   */
  public boolean getStickButtonReleased(final int button) {
    getData();
    if (button <= 0) {
      reportJoystickUnpluggedError();
      return false;
    }
    if (button <= m_joystickButtons.m_count) {
      // If button was released, clear flag and return true
      if ((m_joystickButtonsReleased & 1 << (button - 1)) != 0) {
        m_joystickButtonsReleased &= ~(1 << (button - 1));
        return true;
      } else {
        return false;
      }
    }
    reportJoystickUnpluggedWarning(button, m_port);
    return false;
  }

  /**
   * Get the value of the axis on a joystick. This depends on the mapping of the joystick connected to
   * the specified port.
   *
   * @param axis The analog axis value to read from the joystick.
   * @return The value of the axis on the joystick.
   */
  public double getStickAxis(int axis) {
    getData();
    if (axis < 0 || axis >= HAL.kMaxJoystickAxes) {
      throw new IllegalArgumentException("Joystick axis is out of range");
    }
    if (axis < m_joystickAxes.m_count) {
      return m_joystickAxes.m_axes[axis];
    }
    reportJoystickUnpluggedWarning(axis, m_port);
    return 0.0;
  }

  /**
   * Get the state of a POV on the joystick.
   *
   * @param pov The POV to read.
   * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
   */
  public int getStickPOV(int pov) {
    getData();
    if (pov < 0 || pov >= HAL.kMaxJoystickPOVs) {
      throw new IllegalArgumentException("Joystick POV is out of range");
    }
    if (pov < m_joystickPOVs.m_count) {
      return m_joystickPOVs.m_povs[pov];
    }
    reportJoystickUnpluggedWarning(pov, m_port);
    return -1;
  }

  /**
   * The state of the buttons on the joystick.
   *
   * @return The state of the buttons on the joystick.
   */
  public int getStickButtons() {
    getData();
    return m_joystickButtons.m_buttons;
  }

  protected void getData() {
    // Get the status of all of the joysticks
    byte stick = (byte) m_port;
    m_joystickAxes.m_count = HAL.getJoystickAxes(stick, m_joystickAxes.m_axes);
    m_joystickPOVs.m_count = HAL.getJoystickPOVs(stick, m_joystickPOVs.m_povs);
    m_joystickButtons.m_buttons = HAL.getJoystickButtons(stick, m_buttonCountBuffer);
    m_joystickButtons.m_count = m_buttonCountBuffer.get(0);
  }

  /**
   * Reports errors related to unplugged joysticks Throttles the errors so that they don't overwhelm
   * the DS.
   */
  private static void reportJoystickUnpluggedError(String message) {
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime > m_nextMessageTime) {
      DriverStation.reportError(message, false);
      m_nextMessageTime = currentTime + JOYSTICK_UNPLUGGED_MESSAGE_INTERVAL;
    }
  }

  /**
   * Reports errors related to unplugged joysticks Throttles the errors so that they don't overwhelm
   * the DS.
   */
  private static void reportJoystickUnpluggedWarning(String message) {
    if (DriverStation.isFMSAttached() || !DriverStation.isJoystickConnectionWarningSilenced()) {
      double currentTime = Timer.getFPGATimestamp();
      if (currentTime > m_nextMessageTime) {
        DriverStation.reportWarning(message, false);
        m_nextMessageTime = currentTime + JOYSTICK_UNPLUGGED_MESSAGE_INTERVAL;
      }
    }
  }

  private static void reportJoystickUnpluggedWarning(final int pov, final int stick) {
    reportJoystickUnpluggedWarning("Joystick POV " + pov + " on port " + stick + " not available, check if controller is plugged in");
  }

  private static void reportJoystickUnpluggedError() {
    reportJoystickUnpluggedError("Button indexes begin at 1 in WPILib for C++ and Java\n");
  }
}
