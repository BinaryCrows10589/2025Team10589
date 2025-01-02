package frc.robot.Subsystems.SwerveDrive.Gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    
  @AutoLog
  public static class GyroIOInputs {
    public Rotation2d yawAngle = new Rotation2d();
    public double yawVelocityDegreesPerSecond = 0.0;

  }

  public default void updateInputs(GyroIOInputs inputs, double rotationRateRadiansPerSecond) {}
}