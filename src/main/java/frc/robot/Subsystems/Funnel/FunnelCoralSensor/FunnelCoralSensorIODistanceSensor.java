package frc.robot.Subsystems.Funnel.FunnelCoralSensor;

import com.playingwithfusion.TimeOfFlight;

import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.TransitConstants.TransitCoralSensorConstants;

/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class FunnelCoralSensorIODistanceSensor implements FunnelCoralSensorIO{
    private TimeOfFlight funnleCoralSensor;

    public FunnelCoralSensorIODistanceSensor() {
        this.funnleCoralSensor = new TimeOfFlight(FunnelConstants.kFunnelCoralSensorCANID);
        this.funnleCoralSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    @Override
    public void updateInputs(FunnelCoralSensorIOInputs inputs) {
        inputs.distanceSensorReadingMilameters = this.funnleCoralSensor.getRange();
        inputs.validReading = this.funnleCoralSensor.isRangeValid();
    }
}