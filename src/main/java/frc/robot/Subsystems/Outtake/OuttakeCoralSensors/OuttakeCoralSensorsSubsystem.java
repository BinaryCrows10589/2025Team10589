package frc.robot.Subsystems.Outtake.OuttakeCoralSensors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;

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
        boolean coralDetected = isCoralDetected(this.outtakeCoralSensorsInputs.startValidReading, this.outtakeCoralSensorsInputs.startDistanceSensorReadingMilameters);    
        Logger.recordOutput("Outtake/OuttakeCoralSensors/IsCoralInStart", coralDetected);
        return coralDetected;
    }

    public boolean isCoralInEndOfOuttake() {
        boolean coralDetected = isCoralDetected(this.outtakeCoralSensorsInputs.endValidReading, this.outtakeCoralSensorsInputs.endDistanceSensorReadingMilameters);    
        Logger.recordOutput("Outtake/OuttakeCoralSensors/IsCoralInEnd", coralDetected);
        return coralDetected;
    }

    public boolean isStartReadingValid() { return this.outtakeCoralSensorsInputs.startValidReading; }
    public boolean isEndReadingValid() { return this.outtakeCoralSensorsInputs.endValidReading; }

}
