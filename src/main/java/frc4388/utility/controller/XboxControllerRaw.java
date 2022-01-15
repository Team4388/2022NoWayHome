package frc4388.utility.controller;
import java.nio.ByteBuffer;
import edu.wpi.first.hal.HAL;


public class XboxControllerRaw implements IHandController {
    public static final int LEFT_X_AXIS = 0;
    public static final int LEFT_Y_AXIS = 1;
    public static final int LEFT_TRIGGER_AXIS = 2;
    public static final int RIGHT_TRIGGER_AXIS = 3;
    public static final int RIGHT_X_AXIS = 4;
    public static final int RIGHT_Y_AXIS = 5;
    public static final int LEFT_RIGHT_DPAD_AXIS = 6;
    public static final int TOP_BOTTOM_DPAD_AXIS = 6;
  
    public static final int A_BUTTON = 0;
    public static final int B_BUTTON = 1;
    public static final int X_BUTTON = 2;
    public static final int Y_BUTTON = 3;
    public static final int LEFT_BUMPER_BUTTON = 4;
    public static final int RIGHT_BUMPER_BUTTON = 5;
    public static final int BACK_BUTTON = 6;
    public static final int START_BUTTON = 7;
  
    public static final int LEFT_JOYSTICK_BUTTON = 8;
    public static final int RIGHT_JOYSTICK_BUTTON = 9;
    
    private static final double DEADZONE = 0.08;

  private int m_ID;
  private int m_buttons = 0;
  private int m_numButtons;
  private ByteBuffer m_buttonCountBuffer = ByteBuffer.allocateDirect(1);
  float[] m_axes = new float[HAL.kMaxJoystickAxes];

  public XboxControllerRaw(int id) {
    m_ID = id;
  }


  public void updateInput() {
    HAL.getJoystickAxes((byte) m_ID, m_axes);
    m_buttons = HAL.getJoystickButtons((byte) m_ID, m_buttonCountBuffer);
    m_numButtons = m_buttonCountBuffer.get(0);
  }

  public int getNumButtons() {
    return m_numButtons;
  }

  public boolean getButton(int index) {
      return (m_buttons & 1 << index) > 0;
  }

  /**
   * Checks if the input falls within the deadzone.
   * @param input from an axis on the controller
   * @return true if input falls in deadzone, false if input falls outside deadzone
   */
  public static boolean inDeadZone(double input){
    return (Math.abs(input) < DEADZONE);
  }

  /**
   * Updates an input to have a deadzone around the 0 position
   * @param input from an axis on the controller
   * @return updated input
   */
  private double getAxisWithDeadZoneCheck(double input){
    if(inDeadZone(input)){
      return 0.0;       
    } else {
      return input;
    }
  }

  public double getLeftXAxis() {
      return getAxisWithDeadZoneCheck(m_axes[LEFT_X_AXIS]);
  }

  public double getLeftYAxis() {
    return getAxisWithDeadZoneCheck(m_axes[LEFT_Y_AXIS]);
  }

  public double getRightXAxis() {
    return getAxisWithDeadZoneCheck(m_axes[RIGHT_X_AXIS]);
  }

  public double getRightYAxis() {
    return getAxisWithDeadZoneCheck(m_axes[RIGHT_Y_AXIS]);
  }

  public double getLeftTriggerAxis() {
    return getAxisWithDeadZoneCheck(m_axes[LEFT_TRIGGER_AXIS]);
  }

  public double getRightTriggerAxis() {
    return getAxisWithDeadZoneCheck(m_axes[RIGHT_TRIGGER_AXIS]);
  }

  public int getDpadAngle() {
      return -1;
  }
}