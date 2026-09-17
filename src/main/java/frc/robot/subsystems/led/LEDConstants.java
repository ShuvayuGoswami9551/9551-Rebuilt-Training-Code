package frc.robot.subsystems.led;

public class LEDConstants {
  public static final int CANdleId = 16;
  public static final String CANBus = "Tinivore"; // change if using CANivore

  public static final int length = 30;

  // CANdle indices:
  // 0-7 = onboard LEDs
  // 8+  = external strip
  public static final int ledStartIndex = 8;
  public static final int ledEndIndex = ledStartIndex + length - 1;

  public static final double brightness = 0.5;

  // effect speeds
  public static final double fastStrobeHz = 12.0;
  public static final double autoStrobeHz = 6.0;
  public static final double rainbowHz = 25.0;
  public static final double waveFadeHz = 30.0;
}
