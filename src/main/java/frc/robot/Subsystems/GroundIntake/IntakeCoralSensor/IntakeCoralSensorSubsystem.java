package frc.robot.Subsystems.GroundIntake.IntakeCoralSensor;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.GroundIntakeConstants.IntakeCoralSensorConstants;

/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class IntakeCoralSensorSubsystem extends SubsystemBase{
    private IntakeCoralSensorIO intakeCoralSensorIO;
    private IntakeCoralSensorIOInputsAutoLogged intakeCoralSensorInputs = new IntakeCoralSensorIOInputsAutoLogged();

    public IntakeCoralSensorSubsystem(IntakeCoralSensorIO intakeCoralSensorIO) {
        this.intakeCoralSensorIO = intakeCoralSensorIO;
    }

    @Override
    public void periodic() {
        this.intakeCoralSensorIO.updateInputs(this.intakeCoralSensorInputs);
        Logger.processInputs("GroundIntake/IntakeCoralSensor", this.intakeCoralSensorInputs);
    }

    public boolean isCoralInIntake() {
        return this.intakeCoralSensorInputs.validReading && 
            this.intakeCoralSensorInputs.distanceSensorReadingMilameters <
            IntakeCoralSensorConstants.kIntakeCoralSensorMaxCoralDistanceMilameters;
    }
 

  

    
}