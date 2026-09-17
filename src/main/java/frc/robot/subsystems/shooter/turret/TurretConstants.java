package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class TurretConstants {

  public static final int kTurretID = Ports.kTurret;
  public static final boolean kTurretInverted = false;

  public static final double kTurretGearRatio = 27.0;

  public static final double kTurretRadius = 0.10027; // TODO

  public static final double kP = 130.0; // 100
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.1; // 静态前馈
  public static final double kV = 0.75; // 0.13 轨迹规划得到的目标速度
  public static final double kA = 0.1; // 轨迹规划得到的目标加速度

  // ========== 角度限制 ==========
  public static final double kTurretMinAngle = Units.degreesToRadians(-270); // radius
  public static final double kTurretMaxAngle = Units.degreesToRadians(80); // radius
  public static final double kTurretInitialAngle = Units.degreesToRadians(0);

  public static final double kTurretToleranceDeg = Units.degreesToRadians(3);
  public static final double kTurretFixAngle = Units.degreesToRadians(-90);

  // 炮塔的相对于机器中心的位置
  public static final Translation2d kTurretonRobotoffset = new Translation2d(-0.136598, -0.146088);
  // 摄像头相对于炮塔中心的位置
  public static final Transform3d kCameraonTurretoffset =
      new Transform3d(
          0.184727,
          0.0,
          0.136870,
          new Rotation3d(
              Units.degreesToRadians(0), // roll
              Units.degreesToRadians(-19.727353), // pitch
              Units.degreesToRadians(0))); // yaw

  public static final double kAccelerationRadPerSecSq = Units.degreesToRadians(540.0); // 最大角加速度
  public static final double kVelocityRadPerSec = Units.degreesToRadians(360.0);
  public static final double kMaxWrapVelocity = Units.degreesToRadians(180.0);
  public static final double kMaxTurretTrackingVel = Units.radiansToDegrees(1400.0);
  public static final double kTurretWrapAngle = Units.radiansToDegrees(20.0);
  public static final double kMaxNormalVelocity = Units.radiansToDegrees(1000.0);

  // sim
  public static final double autoStartAngle = Units.degreesToRadians(0.0); // TODO
  public static final double jkMetersSquared = 10.06328;
  public static final double turretLength = 0.20054; // TODO noneed
  public static final double loopPeriodSecs = 0.02;

  public static final double turret_height_meters = 0.3004;
  public static final double kMaxAngularVelocityRadpersec = Units.degreesToRadians(360); // 2 rps
}
