package frc.robot.subsystems.intakearm;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class IntakearmConstants {

  public static final int kLeftIntakeArmId = Ports.kLeftLinearIntake;
  public static final int kRightIntakeArmId = Ports.kRightLinearIntake;
  public static final boolean kLeftIntakeArmInverted = true;
  public static final boolean kRightIntakeArmInverted = false;

  public static final double kIntakearmRadius = 0.241025; // TODO
  public static final double kIntakearmGearRatio = 400.0 / 9.0; // TODO

  public static final double kP = 6.0; // 200.0 // 30.0
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.1
  public static final double kG = 0.0;
  public static final double kS = 0.12; // 0.3
  public static final double kV = 9.0; // 2.0
  public static final double kA = 0.08; // 0.7 // 1.5

  public static final double kAccelerationRadPerSecSq =
      Units.degreesToRadians(175.0); // 最大角加速度 // 750.0
  public static final double kVelocityRadPerSec = Units.degreesToRadians(100.0); // 450.0

  public static final double kIntakearmMinAngle = Units.degreesToRadians(0.0); // TODO:
  public static final double kIntakearmMaxAngle = Units.degreesToRadians(46.0);
  public static final double kIntakearmInitialAngle = Units.degreesToRadians(0.0);
  public static final double kIntakearmDeployAngle = Units.degreesToRadians(45);
  public static final double kIntakearmStowerAngle = Units.degreesToRadians(5);

  public static final double kIntakearmoffset =
      Units.degreesToRadians(0.0); // TODO: offset for Intakearm angle
  public static final double kIntakearmtoleranceDeg = Units.degreesToRadians(1.0);

  public static final double kFixAngle = Units.degreesToRadians(40); // TODO
  public static final double kHoldVoltage = 0.45;
}
