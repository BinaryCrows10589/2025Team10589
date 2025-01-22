package frc.robot.Subsystems.Funnel.FunnelCoralSensor;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelCoralSensorIO {

    @AutoLog
    public static class FunnelCoralSensorIOInputs {
        public double distanceSensorReadingMilameters = 0.0;
        public boolean validReading = false;
    }

    /**
     * Updates all loggable inputes
     * @param inputs FunnelCoralSensorIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(FunnelCoralSensorIOInputs inputs) {}
}
