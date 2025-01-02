package frc.robot.Subsystems.SwerveDrive.Gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;

public class GyroIOPigeon2 implements GyroIO{

    private Pigeon2 gyro;


    public GyroIOPigeon2() {
        configGyro();
    }

    private void configGyro() {
        this.gyro = new Pigeon2(SwerveDriveConstants.kGyroCANID, SwerveDriveConstants.kCANLoopName);
        this.gyro.getConfigurator().apply(new Pigeon2Configuration());
        this.gyro.reset();
    } 

    @Override
    public void updateInputs(GyroIOInputs inputs, double rotationRate) {
        inputs.yawAngle = this.gyro.getRotation2d();
        inputs.yawVelocityDegreesPerSecond = this.gyro.getAngularVelocityZWorld().getValueAsDouble();
    }
}
