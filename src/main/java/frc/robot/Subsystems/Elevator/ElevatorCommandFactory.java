package frc.robot.Subsystems.Elevator;

import frc.robot.Commands.ElevatorCommands.ElevatorToPositionCommand;
import frc.robot.Commands.ElevatorCommands.MoveElevatorManuallyCommand;
import frc.robot.Commands.ElevatorCommands.PrepareL1BackfeedCommand;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsSubsystem;

public class ElevatorCommandFactory {
    // All Subsystems needed for every command
    private final ElevatorSubsystem elevatorSubsystem;


    public ElevatorCommandFactory(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }

    public ElevatorToPositionCommand createElevatorToBasementCommand() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.BASEMENT),
        ElevatorConstants.kGroundIntakePassoffTolorence,
        elevatorSubsystem);
    }

    public ElevatorToPositionCommand createElevatorToFunnelCommand() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.FUNNEL),
        ElevatorConstants.kElevatorFunnelTolorence,
        elevatorSubsystem);
    }

    public ElevatorToPositionCommand createElevatorToL1Command() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.L1),
        ElevatorConstants.kElevatorScorePositionTolorence,
        elevatorSubsystem);
    }

    public ElevatorToPositionCommand createElevatorToL2Command() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.L2),
        ElevatorConstants.kElevatorScorePositionTolorence,
        elevatorSubsystem);
    }

    public ElevatorToPositionCommand createElevatorToL3Command() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.L3),
        ElevatorConstants.kElevatorScorePositionTolorence,
        elevatorSubsystem);
    }

    public ElevatorToPositionCommand createElevatorToL4Command() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.L4),
        ElevatorConstants.kElevatorScorePositionTolorence,
        elevatorSubsystem);
    }

    public ElevatorToPositionCommand createElevatorToBlueSecondCommand() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.BlueHeight),
        ElevatorConstants.kElevatorScorePositionTolorence,
        elevatorSubsystem);
    }

    public ElevatorToPositionCommand createElevatorToBargeScoreCommand() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.SCORE_ALGAE_BARGE),
        ElevatorConstants.kElevatorScoreBargePositionTolorence,
        elevatorSubsystem);
    }

    public ElevatorToPositionCommand createElevatorToProcessorScoreCommand() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.SCORE_ALGAE_PROCESSOR),
        ElevatorConstants.kElevatorScoreProcessorPositionTolorence,
        elevatorSubsystem);
    }
    
    public ElevatorToPositionCommand createElevatorToGroundIntakeAlgaeCommand() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.GROUND_INTAKE_ALGAE),
        ElevatorConstants.kElevatorScoreProcessorPositionTolorence,
        elevatorSubsystem);
    }

    public ElevatorToPositionCommand createElevatorToReefIntakeAlgaeLowCommand() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.REEF_INTAKE_ALGAE_LOW),
        ElevatorConstants.kElevatorScoreProcessorPositionTolorence,
        elevatorSubsystem);
    }
    public ElevatorToPositionCommand createElevatorToReefIntakeAlgaeHighCommand() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.REEF_INTAKE_ALGAE_HIGH),
        ElevatorConstants.kElevatorScoreProcessorPositionTolorence,
        elevatorSubsystem);
    }

    public MoveElevatorManuallyCommand createMoveElevatorUpCommand() {
        return new MoveElevatorManuallyCommand(ElevatorConstants.kManualMovementUpSpeed, elevatorSubsystem);
    }

    public MoveElevatorManuallyCommand createMoveElevatorDownCommand() {
        return new MoveElevatorManuallyCommand(ElevatorConstants.kManualMovementDownSpeed, elevatorSubsystem);
    }
    
}
