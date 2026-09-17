package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  // CAN Buses
  public static final CANBus kRoboRioCANBus = new CANBus("rio");
  public static final CANBus kCANivoreCANBus = new CANBus("Tinivore");

  // CAN Coders
  public static final int kHangerEncoder = 25;
  public static final int kTurretEncoder = 33;
  public static final int kHoodEncoder = 37;

  // Talon FX IDs
  public static final int kIntakeArm = 113;
  public static final int kLeftLinearIntake = 20;
  public static final int kRightLinearIntake = 21;
  public static final int kIntakeRollers = 22;

  public static final int kRotatorRollers = 26;
  public static final int kIndexerRollers = 30;

  public static final int kTurret = 32;
  public static final int kHood = 36;
  public static final int kLeftFlywheel = 34;
  public static final int kRightFlywheel = 35;

  public static final int kHanger = 24;
}
