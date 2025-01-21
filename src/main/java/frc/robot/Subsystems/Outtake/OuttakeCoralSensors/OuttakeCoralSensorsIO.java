package frc.robot.Subsystems.Outtake.OuttakeCoralSensors;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeCoralSensorsIO {
    @AutoLog
    public static class OuttakeCoralSensorsIOInputs {
        public double startDistanceSensorReadingMilameters = 0.0;
        public double endDistanceSensorReadingMilameters = 0.0;
        public boolean startValidReading = false;
        public boolean endValidReading = false;
    }

    /**
     * Updates all loggable inputes
     * @param inputs FunnelCoralSensorIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(OuttakeCoralSensorsIOInputs inputs) {}
}
