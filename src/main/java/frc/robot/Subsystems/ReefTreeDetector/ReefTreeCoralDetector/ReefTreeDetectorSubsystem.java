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
    private int leftDistenceSensorInvalidCount = 0;
    private int rightDistenceSensorInvalidCount = 0;

    public ReefTreeDetectorSubsystem(ReefTreeDetectorIO reefTreeDetectorIO) {
        this.reefTreeDetectorIO = reefTreeDetectorIO;
    }

    @Override
    public void periodic() {
        this.reefTreeDetectorIO.updateInputs(this.reefTreeDetectorInputs);
        Logger.processInputs("ReefTreeDetector/", this.reefTreeDetectorInputs);
    }

    public double getReefTreeDistanceLeft() {
        return this.reefTreeDetectorInputs.distanceSensorReadingMilametersLeft;
    }

    public double getReefTreeDistanceRight() {
        return this.reefTreeDetectorInputs.distanceSensorReadingMilametersRight;
    }

    private boolean isReefTreeScean(double range) {
        return range <
            ReefTreeDetectorConstants.kMaxReefTreeDistance;
    }

    public boolean isInLeftSensorInRange() {
        if(this.reefTreeDetectorInputs.validReadingLeft) {
            this.leftDistenceSensorInvalidCount = 0;
        } else {
            this.leftDistenceSensorInvalidCount++;
        }
        if(this.leftDistenceSensorInvalidCount > ReefTreeDetectorConstants.kMaxInvalidCount) {
            return false;
        } else {
            return true;//isReefTreeScean(this.reefTreeDetectorInputs.distanceSensorReadingMilametersLeft);
        }
    }

    public boolean isInRightSensorInRange() {
        if(this.reefTreeDetectorInputs.validReadingRight) {
            this.rightDistenceSensorInvalidCount = 0;
        } else {
            this.rightDistenceSensorInvalidCount++;
        }
        if(this.rightDistenceSensorInvalidCount > ReefTreeDetectorConstants.kMaxInvalidCount) {
            return false;
        } else {
            return true;//isReefTreeScean(this.reefTreeDetectorInputs.distanceSensorReadingMilametersRight);
        }
    }
 

  

    
}