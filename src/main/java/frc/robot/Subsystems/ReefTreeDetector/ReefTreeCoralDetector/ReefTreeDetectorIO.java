package frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector;

import org.littletonrobotics.junction.AutoLog;

public interface ReefTreeDetectorIO {

    @AutoLog
    public static class ReefTreeDetectorIOInputs {
        public double distanceSensorReadingMilametersLeft = 0.0;
        public double distanceSensorReadingMilametersRight = 0.0;
        public boolean validReadingRight = false;
        public boolean validReadingLeft = false;

    }

    /**
     * Updates all loggable inputes
     * @param inputs ReefTreeDetectorIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(ReefTreeDetectorIOInputs inputs) {}
}
