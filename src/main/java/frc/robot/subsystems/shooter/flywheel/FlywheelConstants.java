package frc.robot.subsystems.shooter.flywheel;

import frc.robot.Ports;

public class FlywheelConstants {
  public static final int kLeftFlywheelId = Ports.kLeftFlywheel;
  public static final int kRightFlywheelId = Ports.kRightFlywheel;
  public static boolean kLeftFlywheelInverted = false;
  public static boolean kRightFlywheelInverted = true;

  public static final double kFlywheelRadius = 0.05; // TODO
  public static final double kFlywheelGearRatio = 0.75; // TODO

  public static final double kP = 4.35; // 10.0
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.001; // 静态前馈
  public static final double kV = 0.15; // 轨迹规划得到的目标速度
  public static final double kA = 0.125; // 轨迹规划得到的目标加速度

  public static final double kVelocityTolerance = 10.0;

  public static final double kAccelerationRadPerSecSq = 180.0; // 最大角加速度
  public static final double kVelocityRadPerSec = 180.0;

  public static final double kMaxVelocity = 725.0;
  public static final double kFixVelocity = 650.0;
  public static final double kPassVelocity = 350.0;
  public static final double kOutTakeVelocity = -200.0;
  public static final double kActiveRatio = 0.35;
  public static final double kTestVelocity = kPassVelocity;
}
