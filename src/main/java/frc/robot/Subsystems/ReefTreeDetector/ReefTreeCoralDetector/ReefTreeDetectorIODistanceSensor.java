package frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector;

import com.playingwithfusion.TimeOfFlight;

import frc.robot.Constants.MechanismConstants.ReefTreeDetectorConstants;
import frc.robot.Constants.MechanismConstants.TransitConstants.TransitCoralSensorConstants;

/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class ReefTreeDetectorIODistanceSensor implements ReefTreeDetectorIO{
    private TimeOfFlight reefTreeDetectorLeft;
    private TimeOfFlight reefTreeDetectorRight;

    public ReefTreeDetectorIODistanceSensor() {
        this.reefTreeDetectorLeft = new TimeOfFlight(ReefTreeDetectorConstants.kReefTreeDetectorLeftCANID);
        this.reefTreeDetectorRight = new TimeOfFlight(ReefTreeDetectorConstants.kReefTreeDetectorRightCANID);
        this.reefTreeDetectorLeft.setRangingMode(TimeOfFlight.RangingMode.Short, 50);
        this.reefTreeDetectorRight.setRangingMode(TimeOfFlight.RangingMode.Short, 50);

    }

    @Override
    public void updateInputs(ReefTreeDetectorIOInputs inputs) {
        inputs.distanceSensorReadingMilametersLeft = this.reefTreeDetectorLeft.getRange();
        inputs.distanceSensorReadingMilametersRight = this.reefTreeDetectorRight.getRange();
        inputs.validReadingLeft = this.reefTreeDetectorLeft.isRangeValid();
        inputs.validReadingRight = this.reefTreeDetectorRight.isRangeValid();

    } 
}