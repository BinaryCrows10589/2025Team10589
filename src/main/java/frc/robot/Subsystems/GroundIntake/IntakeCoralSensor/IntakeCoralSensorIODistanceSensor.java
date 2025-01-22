package frc.robot.Subsystems.GroundIntake.IntakeCoralSensor;

import com.playingwithfusion.TimeOfFlight;

import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.IntakeCoralSensorConstants;

/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class IntakeCoralSensorIODistanceSensor implements IntakeCoralSensorIO{
    private TimeOfFlight intakeCoralSensor;

    public IntakeCoralSensorIODistanceSensor() {
        this.intakeCoralSensor = new TimeOfFlight(IntakeCoralSensorConstants.kIntakeCoralSensorID);
        this.intakeCoralSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    @Override
    public void updateInputs(IntakeCoralSensorIOInputs inputs) {
        inputs.distanceSensorReadingMilameters = this.intakeCoralSensor.getRange();
        inputs.validReading = this.intakeCoralSensor.isRangeValid();
    }
}