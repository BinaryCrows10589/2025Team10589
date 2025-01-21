package frc.robot.Subsystems.Outtake.OuttakeCoralSensors;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeCoralSensorsSubsystem extends SubsystemBase {
    private OuttakeCoralSensorsIO outtakeCoralSensorsIO;
    private OuttakeCoralSensorsIOInputsAutoLogged outtakeCoralSensorsInputs = new OuttakeCoralSensorsIOInputsAutoLogged();

    public OuttakeCoralSensorsSubsystem(OuttakeCoralSensorsIO outtakeCoralSensorsIO) {
        this.outtakeCoralSensorsIO = outtakeCoralSensorsIO;
    }

    @Override
    public void periodic() {
        this.outtakeCoralSensorsIO.updateInputs(outtakeCoralSensorsInputs);
        Logger.processInputs("Outtake/OuttakeCoralSensors", outtakeCoralSensorsInputs);
    }

    private boolean isCoralDetected(boolean validReading, double distanceSensorReadingMillimeters) {
        return
            validReading &&
            distanceSensorReadingMillimeters < OuttakeConstants.kOuttakeCoralSensorMaxCoralDistanceMillimeters;
    }

    public boolean isCoralInStartOfOuttake() {
        return isCoralDetected(this.outtakeCoralSensorsInputs.startValidReading, this.outtakeCoralSensorsInputs.startDistanceSensorReadingMilameters);    
    }

    public boolean isCoralInEndOfOuttake() {
        return isCoralDetected(this.outtakeCoralSensorsInputs.endValidReading, this.outtakeCoralSensorsInputs.endDistanceSensorReadingMilameters);    
    }

}
