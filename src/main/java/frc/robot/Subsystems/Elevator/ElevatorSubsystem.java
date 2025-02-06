package frc.robot.Subsystems.Elevator;

import java.util.Base64;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.PivotContants;
import frc.robot.Utils.GeneralUtils.Tolerance;

public class ElevatorSubsystem extends SubsystemBase {
    ElevatorIO elevatorIO;
    ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    ElevatorMode currentMode = ElevatorMode.AUTOMATIC_POSITIONING;

    public static enum ElevatorPosition {
        BASEMENT,
        L1,
        L2,
        L3,
        L4,
        FUNNEL,
        GROUND_INTAKE_ALGAE,
        SCORE_ALGAE_PROCESSOR,
        SCORE_ALGAE_BARGE,
        REEF_INTAKE_ALGAE
    }

    public static enum ElevatorMode {
        MANUAL,
        AUTOMATIC_POSITIONING
    }

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public void periodic() {
        
        this.elevatorIO.updateInputs(this.elevatorInputs);
        Logger.processInputs("Elevator/", elevatorInputs);
    }

    public void setDesiredElevatorPosition(double desiredPosition) {
        updateElevatorControlMode(ElevatorMode.AUTOMATIC_POSITIONING);
        elevatorIO.setDesiredPosition(desiredPosition);
    }

    public void setDesiredElevatorPosition(ElevatorPosition desiredPosition) {
        updateElevatorControlMode(ElevatorMode.AUTOMATIC_POSITIONING);

        elevatorIO.setDesiredPosition(resolveElevatorPosition(desiredPosition));
    }

    public void incrementDesiredElevatorPosition(double increment) {
        updateElevatorControlMode(ElevatorMode.MANUAL);
        elevatorIO.incrementDesiredPosition(increment);
    }

    public boolean isElevatorInTolorence(double toloranceRotations) {
        return Tolerance.inTolorance(this.elevatorInputs.elevatorRawPosition, this.elevatorInputs.offsetDesiredElevatorPosition,
        toloranceRotations);
    }

    public void updateElevatorControlMode(ElevatorMode controlMode) {
        this.currentMode = controlMode;
    }
    public ElevatorMode getControlMode() {
        return this.currentMode;
    }

    public static double resolveElevatorPosition(ElevatorPosition desiredPosition) {
        switch (desiredPosition) {
            case BASEMENT:
                return 0;
            case L1:
                return 0;
            case L2:
                return .7;
            case L3:
                return .5;
            case L4:
                return .6;

            case FUNNEL:
                return 0;
            case GROUND_INTAKE_ALGAE:
                return 0;
            case REEF_INTAKE_ALGAE:
                return 0;
            case SCORE_ALGAE_PROCESSOR:
                return 0;
            case SCORE_ALGAE_BARGE:
                return 0;    
            default:
                return 0;
        }
    }
}
