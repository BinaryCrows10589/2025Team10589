package frc.robot.Subsystems.SwerveDrive.Gyro;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.MechanismConstants.DrivetrainConstants;
import frc.robot.Constants.MechanismConstants.DrivetrainConstants.SwerveDriveConstants;

public class GyroIOPigeon2 implements GyroIO{

    private Pigeon2 gyro;
    private Rotation2d previousGyroValue;

    public GyroIOPigeon2() {
        configGyro();
    }

    private void configGyro() {
        this.gyro = new Pigeon2(SwerveDriveConstants.kGyroCANID, SwerveDriveConstants.kCANLoopName);
        this.gyro.getConfigurator().apply(new Pigeon2Configuration());
        this.gyro.reset();
        previousGyroValue = gyro.getRotation2d();
        Logger.recordOutput("Gyro/FixedGyro", false);
    } 

    @Override
    public void updateInputs(GyroIOInputs inputs, double rotationRate) {

        inputs.yawAngle = this.gyro.getRotation2d();
        if (Math.abs(previousGyroValue.minus(inputs.yawAngle).getDegrees()) > DrivetrainConstants.SwerveDriveConstants.kMaxAngleDeltaPerFrameDegrees) {
            Logger.recordOutput("Gyro/DetlaAngle", Math.abs(previousGyroValue.minus(inputs.yawAngle).getDegrees()));
            this.gyro.setYaw(previousGyroValue.getDegrees());
            Logger.recordOutput("Gyro/FixedGyro", true);
        } else {
            previousGyroValue = inputs.yawAngle;
        }
        inputs.yawVelocityDegreesPerSecond = this.gyro.getAngularVelocityZWorld().getValueAsDouble();
        Logger.recordOutput("Gyro/IsConnected", this.gyro.isConnected(.5));
    }

    @Override
    public void resetAngle(Rotation2d newZero) {
        this.gyro.setYaw(newZero.getDegrees());
        previousGyroValue = gyro.getRotation2d();
    }
}
