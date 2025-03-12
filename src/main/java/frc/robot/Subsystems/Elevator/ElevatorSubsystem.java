package frc.robot.Subsystems.Elevator;

import java.util.Base64;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.PivotContants;
import frc.robot.Utils.GeneralUtils.Tolerance;

public class ElevatorSubsystem extends SubsystemBase {
    ElevatorIO elevatorIO;
    ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    ElevatorMode currentMode = ElevatorMode.AUTOMATIC_POSITIONING;

    ElevatorPosition lastAssignedDesiredPosition = null;

    private final BooleanSupplier coralInFunnel;
    private final BooleanSupplier coralInOuttake;
    private boolean isCoralInTransit = false;
    private int coralTransitFrameCounter = 0; // Counts frames between seeing the coral in the funnel and not seeing it in the outtake
    private final int coralTransitFrameTimeout = 20 * 10; // Frames until elevator unlocks itself if no coral is detected

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
        REEF_INTAKE_ALGAE_LOW,
        REEF_INTAKE_ALGAE_HIGH
    }

    public static enum ElevatorMode {
        MANUAL,
        AUTOMATIC_POSITIONING
    }

    public ElevatorSubsystem(ElevatorIO elevatorIO, BooleanSupplier coralInFunnel, BooleanSupplier coralInOuttake) {
        this.elevatorIO = elevatorIO;
        this.coralInFunnel = coralInFunnel;
        this.coralInOuttake = coralInOuttake;
    }

    public void periodic() {
        
        this.elevatorIO.updateInputs(this.elevatorInputs);
        Logger.processInputs("Elevator/", elevatorInputs);

        if (coralInFunnel.getAsBoolean()) {
            coralTransitFrameCounter = 0;
            this.isCoralInTransit = true;
        }
        else if (coralInOuttake.getAsBoolean()) {
            coralTransitFrameCounter = 0;
            this.isCoralInTransit = false;
        } else {
            if (this.isCoralInTransit) {
                coralTransitFrameCounter++;
                if (coralTransitFrameCounter > coralTransitFrameTimeout) {
                    this.isCoralInTransit = false; // Unlock elevator
                }
            }
        }
    }

    public void disableElevatorMotors() {
        this.elevatorIO.disableElevatorMotors();
    }

    public void setDesiredElevatorPosition(double desiredPosition) {
        if (this.isCoralInTransit) return;
        lastAssignedDesiredPosition = null;
       
        elevatorIO.setDesiredPosition(desiredPosition);
    }

    public void setDesiredElevatorPosition(ElevatorPosition desiredPosition) {
        if (this.isCoralInTransit) return;
        lastAssignedDesiredPosition = desiredPosition;

        //elevatorIO.setDesiredPosition(resolveElevatorPosition(desiredPosition));
    }

    public void incrementDesiredElevatorPosition(double increment) {
        elevatorIO.incrementDesiredPosition(increment);
    }

    public boolean isElevatorInTolorence(double toloranceRotations) {
        return Tolerance.inTolorance(this.elevatorInputs.elevatorRawPosition, this.elevatorInputs.offsetDesiredElevatorPosition,
        toloranceRotations);
    }

    public ElevatorPosition getDesiredElevatorPosition() {
        return lastAssignedDesiredPosition;
    }

    public double getCurrentElevatorPosition() {
        return this.elevatorInputs.elevatorOffsetPosition;
    }

    public static double resolveElevatorPosition(ElevatorPosition desiredPosition) {
        switch (desiredPosition) {
            case BASEMENT:
                return 0.0;
            case L1://.255
            ////.75
                return .295;//.275;//.295;
            case L2:
                return .38;//.36;//.380;
            case L3:
                return .56;//.55;//.56;
            case L4:
                return .835;//.842;//.8325;//.8375;//.854;//.864;
            case FUNNEL:
                return 0;
            case GROUND_INTAKE_ALGAE:
                return 0;
            case REEF_INTAKE_ALGAE_LOW:
                return .315;//.33;//.349;
            case REEF_INTAKE_ALGAE_HIGH:
                return .52;//.51; //.52
                
            case SCORE_ALGAE_PROCESSOR:
                return 0;
            case SCORE_ALGAE_BARGE:
                return 0.864;    
            default:
                return 0;
        }
    }
}
