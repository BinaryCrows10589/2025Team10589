package frc.robot.Subsystems.GroundIntake.IntakeCoralSensor;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIO.SwerveModuleIOInputs;

public interface IntakeCoralSensorIO {

    @AutoLog
    public static class IntakeCoralSensorIOInputs {
        public double distanceSensorReadingMilameters = 0.0;
        public boolean validReading = false;
    }

    /**
     * Updates all loggable inputes
     * @param inputs CoralSensorIntakeIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(IntakeCoralSensorIOInputs inputs) {}
}
