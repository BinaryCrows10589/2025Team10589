package frc.robot.Subsystems.Outtake.OuttakeCoralSensors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;

public class OuttakeCoralSensorsSubsystem extends SubsystemBase {
    private OuttakeCoralSensorsIO outtakeCoralSensorsIO;
    private OuttakeCoralSensorsIOInputsAutoLogged outtakeCoralSensorsInputs = new OuttakeCoralSensorsIOInputsAutoLogged();
    private double lastStartReading = -1;
    private double lastEndReading = -1;

    public OuttakeCoralSensorsSubsystem(OuttakeCoralSensorsIO outtakeCoralSensorsIO) {
        this.outtakeCoralSensorsIO = outtakeCoralSensorsIO;
    }

    @Override
    public void periodic() {
        this.outtakeCoralSensorsIO.updateInputs(outtakeCoralSensorsInputs);
        Logger.processInputs("Outtake/OuttakeCoralSensors", outtakeCoralSensorsInputs);
    }
    
    private boolean isCoralDetected(double distanceSensorReadingMillimeters) {
        return distanceSensorReadingMillimeters < OuttakeConstants.kOuttakeCoralSensorMaxCoralDistanceMillimeters;
    }

    public boolean isCoralInStartOfOuttake(boolean defualtValue) {
        double reading = 0; 
        if(this.outtakeCoralSensorsInputs.startValidReading) {
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
        Logger.recordOutput("Outtake/OuttakeCoralSensors/IsCoralInEnd", coralDetected);
        return coralDetected;
    }

    public boolean isStartReadingValid() { return this.outtakeCoralSensorsInputs.startValidReading; }
    public boolean isEndReadingValid() { return this.outtakeCoralSensorsInputs.endValidReading; }

}
