package frc.robot.Subsystems.SwerveDrive.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class GyroIOSim implements GyroIO {
    double lastInputsUpdateTime = Timer.getFPGATimestamp();
    
    public GyroIOSim() {
    }

    @Override
    public void updateInputs(GyroIOInputs inputs, double rotationRateRadiansPerSecond) {
        double currentTime = Timer.getFPGATimestamp();  
        double deltaTimeSecond = currentTime - this.lastInputsUpdateTime;
        this.lastInputsUpdateTime = currentTime;

        inputs.yawAngle = Rotation2d.fromRadians(((rotationRateRadiansPerSecond * deltaTimeSecond) + inputs.yawAngle.getRadians()));
        inputs.yawVelocityDegreesPerSecond = Units.radiansToDegrees(rotationRateRadiansPerSecond);   
    }
}

