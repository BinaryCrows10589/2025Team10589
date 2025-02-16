package frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants.ReefTreeDetectorConstants;


/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class ReefTreeDetectorSubsystem extends SubsystemBase{
    private ReefTreeDetectorIO reefTreeDetectorIO;
    private ReefTreeDetectorIOInputsAutoLogged reefTreeDetectorInputs = new ReefTreeDetectorIOInputsAutoLogged();
    private double lastValidReading = -1;

    public ReefTreeDetectorSubsystem(ReefTreeDetectorIO reefTreeDetectorIO) {
        this.reefTreeDetectorIO = reefTreeDetectorIO;
    }

    @Override
    public void periodic() {
        this.reefTreeDetectorIO.updateInputs(this.reefTreeDetectorInputs);
        Logger.processInputs("ReefTreeDetector/", this.reefTreeDetectorInputs);
    }

    public double getReefTreeDistance() {
        return this.reefTreeDetectorInputs.distanceSensorReadingMilameters;
    }

    private boolean isReefTreeScean(double range) {
        return range <
            ReefTreeDetectorConstants.kMaxReefTreeDistance;
    }

    public boolean isReefTreeInRange(boolean defualtValue) {
        double reading = 0; 
        reading = this.reefTreeDetectorInputs.distanceSensorReadingMilameters;
            
        boolean coralDetected = reading == -1 ? defualtValue : isReefTreeScean(reading);  
        Logger.recordOutput("Outtake/OuttakeCoralSensors/IsCoralInStart", coralDetected);
        return coralDetected;
    }
 

  

    
}