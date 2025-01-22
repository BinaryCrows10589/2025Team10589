package frc.robot.Subsystems.TransitTunnel.TransitCoralSensor;

import com.playingwithfusion.TimeOfFlight;

import frc.robot.Constants.MechanismConstants.TransitConstants.TransitCoralSensorConstants;

/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class TransitCoralSensorIODistanceSensor implements TransitCoralSensorIO{
    private TimeOfFlight transitCoralSensor;

    public TransitCoralSensorIODistanceSensor() {
        this.transitCoralSensor = new TimeOfFlight(TransitCoralSensorConstants.kTransitCoralSensorCANID);
        this.transitCoralSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    @Override
    public void updateInputs(TransitCoralSensorIOInputs inputs) {
        inputs.distanceSensorReadingMilameters = this.transitCoralSensor.getRange();
        inputs.validReading = this.transitCoralSensor.isRangeValid();
    }
}