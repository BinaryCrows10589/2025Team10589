package frc.robot.Subsystems.Outtake.OuttakeCoralSensors;

import com.playingwithfusion.TimeOfFlight;

import frc.robot.Constants.MechanismConstants.OuttakeConstants;

public class OuttakeCoralSensorsIODistanceSensor implements OuttakeCoralSensorsIO {
    private TimeOfFlight outtakeCoralSensorStart;
    private TimeOfFlight outtakeCoralSensorEnd;

    public OuttakeCoralSensorsIODistanceSensor() {
        this.outtakeCoralSensorStart = new TimeOfFlight(OuttakeConstants.kOuttakeCoralSensorStartCANID);
        this.outtakeCoralSensorEnd = new TimeOfFlight(OuttakeConstants.kOuttakeCoralSensorEndCANID);

        this.outtakeCoralSensorStart.setRangingMode(TimeOfFlight.RangingMode.Short, 30);
        this.outtakeCoralSensorEnd.setRangingMode(TimeOfFlight.RangingMode.Short, 30);
    }

    @Override
    public void updateInputs(OuttakeCoralSensorsIOInputs inputs) {
        inputs.startDistanceSensorReadingMilameters = this.outtakeCoralSensorStart.getRange();
        inputs.endDistanceSensorReadingMilameters = this.outtakeCoralSensorEnd.getRange();
        inputs.startValidReading = this.outtakeCoralSensorStart.isRangeValid();
        inputs.endValidReading = this.outtakeCoralSensorEnd.isRangeValid();
    }
}
