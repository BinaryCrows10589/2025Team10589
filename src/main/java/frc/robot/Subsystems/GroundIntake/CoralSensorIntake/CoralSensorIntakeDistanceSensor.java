package frc.robot.Subsystems.GroundIntake.CoralSensorIntake;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Subsystems.GroundIntake.CoralSensorIntake.CoralSensorIntakeIO.CoralSensorIntakeIOInputs;

/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class CoralSensorIntakeDistanceSensor implements CoralSensorIntakeIO{
    private TimeOfFlight coralSensorIntake;

    public CoralSensorIntakeDistanceSensor() {
        this.coralSensorIntake = new TimeOfFlight(GroundIntakeConstants.CoralSensorIntakeID);
        this.coralSensorIntake.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    @Override
    public void updateInputs(CoralSensorIntakeIOInputs inputs) {
        inputs.distanceSensorReadingMilameters = this.coralSensorIntake.getRange();
        inputs.validReading = this.coralSensorIntake.isRangeValid();
    }
}