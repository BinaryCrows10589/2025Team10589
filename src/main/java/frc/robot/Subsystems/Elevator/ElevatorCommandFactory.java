package frc.robot.Subsystems.Elevator;

import java.util.function.Supplier;

import frc.robot.Commands.ElevatorCommands.ElevatorToPositionCommand;
import frc.robot.Commands.ElevatorCommands.MoveElevatorManuallyCommand;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.JoystickUtils.ButtonBoardInterface;
import frc.robot.Utils.JoystickUtils.ControllerInterface;

public class ElevatorCommandFactory {
    // All Subsystems needed for every command
    private final ElevatorSubsystem elevatorSubsystem;

    // All other dependencies for every command that uses primarly DriveSubsystem
    private final ControllerInterface mechanismController;

    public ElevatorCommandFactory(ElevatorSubsystem elevatorSubsystem, ControllerInterface mechanismController) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.mechanismController = mechanismController;
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

    public ElevatorToPositionCommand createElevatorToReefIntakeAlgaeCommand() {
        return new ElevatorToPositionCommand(ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.REEF_INTAKE_ALGAE),
        ElevatorConstants.kElevatorScoreProcessorPositionTolorence,
        elevatorSubsystem);
    }

    public MoveElevatorManuallyCommand createMoveElevatorUpCommand(Supplier<Boolean> condition) {
        return new MoveElevatorManuallyCommand(ElevatorConstants.kManualMovementUpSpeed, condition, elevatorSubsystem);
    }

    public MoveElevatorManuallyCommand createMoveElevatorDownCommand(Supplier<Boolean> condition) {
        return new MoveElevatorManuallyCommand(ElevatorConstants.kManualMovementDownSpeed, condition, elevatorSubsystem);
    }
    
}
