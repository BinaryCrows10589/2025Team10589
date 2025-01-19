package frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector;

import com.playingwithfusion.TimeOfFlight;

import frc.robot.Constants.TransitConstants.TransitCoralSensorConstants;

/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class ReefTreeDetectorIODistanceSensor implements ReefTreeDetectorIO{
    private TimeOfFlight reefTreeDetector;

    public ReefTreeDetectorIODistanceSensor() {
        this.reefTreeDetector = new TimeOfFlight(TransitCoralSensorConstants.kTransitCoralSensorCANID);
        this.reefTreeDetector.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    @Override
    public void updateInputs(ReefTreeDetectorIOInputs inputs) {
        inputs.distanceSensorReadingMilameters = this.reefTreeDetector.getRange();
        inputs.validReading = this.reefTreeDetector.isRangeValid();
    }
}