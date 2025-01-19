package frc.robot.Subsystems.TransitTunnel.TransitCoralSensor;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransitConstants.TransitCoralSensorConstants;
import frc.robot.Subsystems.TransitTunnel.CoralSensorTransit.CoralSensorTransitIOInputsAutoLogged;


/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class TransitCoralSensorSubsystem extends SubsystemBase{
    private TransitCoralSensorIO TransitCoralSensorIO;
    private CoralSensorTransitIOInputsAutoLogged transitCoralSensorInputs = new CoralSensorTransitIOInputsAutoLogged();

    public TransitCoralSensorSubsystem(TransitCoralSensorIO TransitCoralSensorIO) {
        this.TransitCoralSensorIO = TransitCoralSensorIO;
    }

    @Override
    public void periodic() {
        this.TransitCoralSensorIO.updateInputs(this.transitCoralSensorInputs);
        Logger.processInputs("Transit/CoralSensorTransit", this.transitCoralSensorInputs);
    }

    public boolean isCoralInTransit() {
        return this.transitCoralSensorInputs.validReading && 
            this.transitCoralSensorInputs.distanceSensorReadingMilameters <
            TransitCoralSensorConstants.kTransitCoralSensorMaxCoralDistanceMilameters;
    }
 

  

    
}