package frc.robot.Subsystems.Outtake.OuttakeCoralSensors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;

public class OuttakeCoralSensorsSubsystem extends SubsystemBase {
    private OuttakeCoralSensorsIO outtakeCoralSensorsIO;
    private OuttakeCoralSensorsIOInputsAutoLogged outtakeCoralSensorsInputs = new OuttakeCoralSensorsIOInputsAutoLogged();
    private double lastStartReading = -1;
    private double lastEndReading = -1;

    private boolean initializedReadingValidity = false;

    private boolean isStartValid;
    private boolean isEndValid;

    private int startSensorContradictionCount = 0;
    private int endSensorContradictionCount = 0;

    private final int maxSensorContradictions = 25;

    public OuttakeCoralSensorsSubsystem(OuttakeCoralSensorsIO outtakeCoralSensorsIO) {
        this.outtakeCoralSensorsIO = outtakeCoralSensorsIO;
    }

    @Override
    public void periodic() {

        if (!initializedReadingValidity) {
            initializedReadingValidity = true;
            isStartValid = this.outtakeCoralSensorsInputs.startValidReading;
            isEndValid = this.outtakeCoralSensorsInputs.endValidReading;
        }
        
        this.outtakeCoralSensorsIO.updateInputs(outtakeCoralSensorsInputs);
        Logger.processInputs("Outtake/OuttakeCoralSensors", outtakeCoralSensorsInputs);

        if (outtakeCoralSensorsInputs.startValidReading != isStartValid) {
            startSensorContradictionCount++;
            if (startSensorContradictionCount > maxSensorContradictions) {
                isStartValid = outtakeCoralSensorsInputs.startValidReading;
                startSensorContradictionCount = 0;
            }
        } else {
            startSensorContradictionCount = 0;
        }

        if (outtakeCoralSensorsInputs.endValidReading != isEndValid) {
            endSensorContradictionCount++;
            if (endSensorContradictionCount > maxSensorContradictions) {
                isEndValid = outtakeCoralSensorsInputs.endValidReading;
                endSensorContradictionCount = 0;
            }
        } else {
            endSensorContradictionCount = 0;
        }
    }
    
    private boolean isCoralDetected(double distanceSensorReadingMillimeters) {
        return distanceSensorReadingMillimeters < OuttakeConstants.kOuttakeCoralSensorMaxCoralDistanceMillimeters;
    }

    public boolean isCoralInStartOfOuttake(boolean defualtValue) {
        double reading = 0; 
        if(isStartValid) {
            reading = this.outtakeCoralSensorsInputs.startDistanceSensorReadingMilameters;
            this.lastStartReading = reading;
        } else {
            reading = lastStartReading;
        }

        boolean coralDetected = reading == -1 ? defualtValue : isCoralDetected(reading);  
        Logger.recordOutput("Outtake/OuttakeCoralSensors/IsCoralInStart", coralDetected);
        return coralDetected;
    }

    public boolean isCoralOutOfElevator() {
        return isCoralInStartOfOuttake(false);
    }

    public boolean isCoralInEndOfOuttake(boolean defualtValue) {
        double reading = 0; 
        if(this.outtakeCoralSensorsInputs.endValidReading) {
            reading = this.outtakeCoralSensorsInputs.endDistanceSensorReadingMilameters;
            this.lastEndReading = reading;
        } else {
            reading = lastEndReading;
        }

        boolean coralDetected = reading == -1 ? defualtValue : isCoralDetected(reading);  
        if(!this.outtakeCoralSensorsInputs.startValidReading) {
            coralDetected = defualtValue;
        }
            */
        boolean coralDetected = isCoralDetected(this.outtakeCoralSensorsInputs.endDistanceSensorReadingMilameters) && (
        defualtValue ? !this.isEndValid : this.isEndValid);
        Logger.recordOutput("Outtake/OuttakeCoralSensors/IsCoralInEnd", coralDetected);
        return coralDetected;
    }

    public boolean isStartReadingValid() { return this.outtakeCoralSensorsInputs.startValidReading; }
    public boolean isEndReadingValid() { return this.outtakeCoralSensorsInputs.endValidReading; }

}