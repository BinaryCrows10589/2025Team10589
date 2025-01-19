package frc.robot.Subsystems.Funnel.FunnelCoralSensor;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.TransitConstants.TransitCoralSensorConstants;


/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class FunnelCoralSensorSubsystem extends SubsystemBase{
    private FunnelCoralSensorIO funnelCoralSensorIO;
    private FunnelCoralSensorIOInputsAutoLogged funnelCoralSensorInputs = new FunnelCoralSensorIOInputsAutoLogged();

    public FunnelCoralSensorSubsystem(FunnelCoralSensorIO transitCoralSensorIO) {
        this.funnelCoralSensorIO = transitCoralSensorIO;
    }

    @Override
    public void periodic() {
        this.funnelCoralSensorIO.updateInputs(this.funnelCoralSensorInputs);
        Logger.processInputs("Funnel/FunnelCoralSensor", this.funnelCoralSensorInputs);
    }

    public boolean isCoralInTransit() {
        return this.funnelCoralSensorInputs.validReading && 
            this.funnelCoralSensorInputs.distanceSensorReadingMilameters <
            FunnelConstants.kFunnelCoralSensorMaxCoralDistanceMilameters;
    }
 

  

    
}