package frc.robot.Subsystems.GroundIntake.CoralSensorIntake;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.GroundIntakeConstants.CoralSensorIntakeConstants;

/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class CoralSensorIntakeSubsystem extends SubsystemBase{
    private CoralSensorIntakeIO coralSensorIntakeIO;
    private CoralSensorIntakeIOInputsAutoLogged coralSensorIntakeInputs = new CoralSensorIntakeIOInputsAutoLogged();

    public CoralSensorIntakeSubsystem(CoralSensorIntakeIO coralSensorIntakeIO) {
        this.coralSensorIntakeIO = coralSensorIntakeIO;
    }

    @Override
    public void periodic() {
        this.coralSensorIntakeIO.updateInputs(this.coralSensorIntakeInputs);
        Logger.processInputs("GroundIntake/CoralSensorIntake", this.coralSensorIntakeInputs);
    }

    public boolean isCoralInIntake() {
        return this.coralSensorIntakeInputs.validReading && 
            this.coralSensorIntakeInputs.distanceSensorReadingMilameters <
            CoralSensorIntakeConstants.kCoralIntakeSensorMaxCoralDistanceMilameters;
    }
 

  

    
}