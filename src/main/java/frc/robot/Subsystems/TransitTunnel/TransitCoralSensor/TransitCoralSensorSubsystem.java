package frc.robot.Subsystems.TransitTunnel.TransitCoralSensor;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransitConstants.TransitCoralSensorConstants;


/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class TransitCoralSensorSubsystem extends SubsystemBase{
    private TransitCoralSensorIO transitCoralSensorIO;
    private TransitCoralSensorIOInputsAutoLogged transitCoralSensorInputs = new TransitCoralSensorIOInputsAutoLogged();

    public TransitCoralSensorSubsystem(TransitCoralSensorIO transitCoralSensorIO) {
        this.transitCoralSensorIO = transitCoralSensorIO;
    }

    @Override
    public void periodic() {
        this.transitCoralSensorIO.updateInputs(this.transitCoralSensorInputs);
        Logger.processInputs("Transit/TransitCoralSensor", this.transitCoralSensorInputs);
    }

    public boolean isCoralInTransit() {
        return this.transitCoralSensorInputs.validReading && 
            this.transitCoralSensorInputs.distanceSensorReadingMilameters <
            TransitCoralSensorConstants.kTransitCoralSensorMaxCoralDistanceMilameters;
    }
 

  

    
}