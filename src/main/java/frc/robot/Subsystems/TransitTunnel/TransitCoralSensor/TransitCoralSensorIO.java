package frc.robot.Subsystems.TransitTunnel.TransitCoralSensor;

import org.littletonrobotics.junction.AutoLog;

public interface TransitCoralSensorIO {

    @AutoLog
    public static class TransitCoralSensorIOInputs {
        public double distanceSensorReadingMilameters = 0.0;
        public boolean validReading = false;
    }

    /**
     * Updates all loggable inputes
     * @param inputs CoralSensorIntakeIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(TransitCoralSensorIOInputs inputs) {}
}
