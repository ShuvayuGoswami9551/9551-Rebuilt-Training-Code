package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.FieldConstants;
/// import  frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.util.geometry.AllianceFlipUtil;
import frc.robot.util.geometry.GeomUtil;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class ShotCalculator {
  private static ShotCalculator instance;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  public enum ShotTarget {
    Hub,
    up,
    down,
    CUSTOM
  }

  private ShotTarget currentTarget = ShotTarget.Hub;
  private Translation2d customTarget = new Translation2d();
  public Translation2d robotToTurret = TurretConstants.kTurretonRobotoffset;
  //   private Transform3d turretToCamera;

  private Drive drive;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  // 增加一个初始化方法
  public void init(Drive drive) {
    this.drive = drive;
  }

  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  public ShootingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.07;
    maxDistance = 5.08;
    phaseDelay = 0.03;

    // C2C distance from turret to hub, angle of hood
    shotHoodAngleMap.put(1.07, Rotation2d.fromDegrees(0.0));
    shotHoodAngleMap.put(1.5367, Rotation2d.fromDegrees(22.5));
    shotHoodAngleMap.put(1.9558, Rotation2d.fromDegrees(35));
    shotHoodAngleMap.put(2.5273, Rotation2d.fromDegrees(50));
    shotHoodAngleMap.put(3.0194, Rotation2d.fromDegrees(55));
    shotHoodAngleMap.put(3.4735, Rotation2d.fromDegrees(65));
    shotHoodAngleMap.put(4.0297, Rotation2d.fromDegrees(75));
    shotHoodAngleMap.put(4.5720, Rotation2d.fromDegrees(80));
    shotHoodAngleMap.put(5.0800, Rotation2d.fromDegrees(85));
    shotHoodAngleMap.put(5.8293, Rotation2d.fromDegrees(90));

    // C2C distance from turret to hub, speed of flywheel in rad/s
    shotFlywheelSpeedMap.put(1.07, 450.0);
    shotFlywheelSpeedMap.put(1.5367, 450.0);
    shotFlywheelSpeedMap.put(1.9558, 465.0);
    shotFlywheelSpeedMap.put(2.5273, 485.0);
    shotFlywheelSpeedMap.put(3.0194, 515.0);
    shotFlywheelSpeedMap.put(3.4735, 525.0);
    shotFlywheelSpeedMap.put(4.0297, 550.0);
    shotFlywheelSpeedMap.put(4.5720, 575.0);
    shotFlywheelSpeedMap.put(5.0800, 600.0);
    shotFlywheelSpeedMap.put(5.8293, 650.0);

    // C2C distance from turret to hub, time of flight in sec
    timeOfFlightMap.put(1.07, 0.7625);
    timeOfFlightMap.put(1.5367, 0.8667);
    timeOfFlightMap.put(1.9558, 0.9333);
    timeOfFlightMap.put(2.5273, 1.0333);
    timeOfFlightMap.put(3.0194, 0.9333);
    timeOfFlightMap.put(3.4735, 1.0);
    timeOfFlightMap.put(4.0297, 1.0333);
    timeOfFlightMap.put(4.5720, 1.05);
    timeOfFlightMap.put(5.0800, 1.0667);
    timeOfFlightMap.put(5.8293, 1.1);

    passingFlywheelSpeedMap.put(1.5319, 450.0);
    passingFlywheelSpeedMap.put(3.78178, 550.0);
    passingFlywheelSpeedMap.put(6.577605, 650.0);
  }

  //   public void clearShootingParameters() {
  //     latestParameters = null;
  //   }

  public ShootingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    // Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    Pose2d estimatedPose = drive.getPose();
    ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();
    // ChassisSpeeds robotRelativeVelocity = new ChassisSpeeds();

    // Advance pose by phase delay
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    Translation2d target = getCurrentTarget();
    Pose2d turretPosition = estimatedPose.transformBy(robotToTurret.toTransform2d());
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = drive.getFieldVelocity();
    // ChassisSpeeds robotVelocity = new ChassisSpeeds();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = shotHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians(); // 弧度
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new ShootingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestParameters;
  }

  public double getPassingFlywheelSpeed() {
    // 获取当前机器人位置
    Pose2d robotPose = drive.getPose();
    Translation2d target = getCurrentTarget();

    // 计算距离 (直接使用机器人中心到 Hub 的距离)
    double distanceToHub = robotPose.getTranslation().getDistance(target);

    // 查表返回所需速度
    double passingSpeed = passingFlywheelSpeedMap.get(distanceToHub);

    // 记录日志，方便调试插值是否正确
    Logger.recordOutput("ShotCalculator/PassingDistance", distanceToHub);
    Logger.recordOutput("ShotCalculator/PassingFlywheelSpeed", passingSpeed);

    return passingSpeed;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }

  public void setTarget(ShotTarget target) {
    if (this.currentTarget != target) {
      this.currentTarget = target;
      clearShootingParameters(); // 非常重要：清缓存
    }
  }

  public void setCustomTarget(Translation2d target) {
    this.customTarget = target;
    this.currentTarget = ShotTarget.CUSTOM;
    clearShootingParameters();
  }

  private Translation2d getCurrentTarget() {
    return switch (currentTarget) {
      case Hub -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

      case down -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

      case up -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

      case CUSTOM -> customTarget;
    };
  }

  public Translation2d getRobotToTurret() {
    return robotToTurret;
  }

  public static LinearVelocity angularToLinearVelocity(double flywheelSpeed, double d) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'angularToLinearVelocity'");
  }
}
