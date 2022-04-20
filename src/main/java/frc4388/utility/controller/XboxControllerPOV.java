package frc4388.utility.controller;

/** Represents a d-pad angle on an XboxController. */
public enum XboxControllerPOV {
  kUp(0),
  kUpRight(45),
  kRight(90),
  kDownRight(135),
  kDown(180),
  kDownLeft(225),
  kLeft(270),
  kUpLeft(315);

  @SuppressWarnings("MemberName")
  public final int value;

  XboxControllerPOV(int value) {
    this.value = value;
  }

  @Override
  public String toString() {
    return this.name().substring(1) + "Button";
  }
}
