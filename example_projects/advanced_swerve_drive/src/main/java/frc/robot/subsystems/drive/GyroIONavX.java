package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.util.NavX;
import java.util.Queue;

public class GyroIONavX implements GyroIO {
  public static NavX navX;
  private final Queue<Double> yawPositionQueue;

  private static Rotation3d rotation;

  public GyroIONavX() {
    System.out.println("[Init] Creating NavX");

    switch (Constants.getRobot()) {
      case ROBOT_2023C:
      case ROBOT_2023P:
        navX = new NavX(SPI.Port.kMXP);
        break;
      default:
        throw new RuntimeException("Invalid robot for NavX");
    }

    navX.reset();
    navX.resetDisplacement();
    navX.zeroYaw();
    yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> navX.getYaw());
  }

  public void updateInputs(GyroIOInputs inputs) {
    rotation = navX.getRotation3d();
    inputs.connected = navX.isConnected();
    inputs.rollPositionRad = rotation.getX();
    inputs.pitchPositionRad = rotation.getY();
    inputs.yawPositionRad = rotation.getZ();
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(navX.getRawGyroX());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(navX.getRawGyroY());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navX.getRawGyroZ());

    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawPositionQueue.clear();
  }
}
