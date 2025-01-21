package frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector;

import org.littletonrobotics.junction.AutoLog;

public interface ReefTreeDetectorIO {

    @AutoLog
    public static class ReefTreeDetectorIOInputs {
        public double distanceSensorReadingMilameters = 0.0;
        public boolean validReading = false;
    }

    /**
     * Updates all loggable inputes
     * @param inputs ReefTreeDetectorIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(ReefTreeDetectorIOInputs inputs) {}
}
