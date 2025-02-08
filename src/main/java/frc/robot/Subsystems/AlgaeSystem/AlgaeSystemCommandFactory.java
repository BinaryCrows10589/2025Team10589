package frc.robot.Subsystems.AlgaeSystem;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AlgaeCommands.AlgaePivotToPositionCommand;
import frc.robot.Commands.AlgaeCommands.RunAlgaeWheelsCommand;
import frc.robot.Commands.ElevatorCommands.ElevatorToPositionCommand;
import frc.robot.Constants.MechanismConstants.AlgaePivotConstants;
import frc.robot.Constants.MechanismConstants.AlgaeWheelConstants;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotSubsystem;
import frc.robot.Subsystems.AlgaeSystem.AlgaeWheels.AlgaeWheelSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.Utils.CommandUtils.ParallelGroupCommand;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class AlgaeSystemCommandFactory {
    private final AlgaePivotSubsystem algaePivotSubsystem;
    private final AlgaeWheelSubsystem algaeWheelSubsystem;

    public AlgaeSystemCommandFactory(AlgaePivotSubsystem algaePivotSubsystem, AlgaeWheelSubsystem algaeWheelSubsystem) {
        this.algaePivotSubsystem = algaePivotSubsystem;
        this.algaeWheelSubsystem = algaeWheelSubsystem;
    }

    public RunAlgaeWheelsCommand createAlgaeWheelGroundIntakeCommand() {
        return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kGroundIntakeVoltage);
    }

    public RunAlgaeWheelsCommand createAlgaeWheelReefTreeIntakeCommand() {
        return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kReefTreeIntakeVoltage);
    }

    public RunAlgaeWheelsCommand createAlgaeWheelBargeOuttakeCommand() {
        return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kOuttakeBargeVoltage);
    }

    public RunAlgaeWheelsCommand createAlgaeWheelProcessorOuttakeCommand() {
        return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kOuttakeProcessorVoltage);
    }

    public RunAlgaeWheelsCommand createAlgaeWheelOuttakeCommand(ElevatorSubsystem elevatorSubsystem) {
        ElevatorPosition elevatorPosition = elevatorSubsystem.getDesiredElevatorPosition();

        if (elevatorPosition == ElevatorPosition.SCORE_ALGAE_BARGE) {
            return createAlgaeWheelBargeOuttakeCommand();
        } else if (elevatorPosition == ElevatorPosition.SCORE_ALGAE_PROCESSOR) {
            return createAlgaeWheelProcessorOuttakeCommand();
        } else {
            return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kOuttakeDefaultVoltage);
        }
    }

    public AlgaePivotToPositionCommand createAlgaePivotToGroundIntakePositionCommand() {
        return new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kGroundIntakePositionRotations);
    }

    public AlgaePivotToPositionCommand createAlgaePivotToReefTreeIntakePositionCommand() {
        return new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kReefTreeIntakePositionRotations);
    }

    public ParallelGroupCommand createAlgaePivotToReefTreeWithElevatorIntakePositionCommand(ElevatorToPositionCommand elevatorToPositionCommand) {
        return new ParallelGroupCommand(
            createAlgaePivotToReefTreeIntakePositionCommand(),
            elevatorToPositionCommand
        );
    }

    public AlgaePivotToPositionCommand createAlgaePivotToProcessorOuttakePositionCommand() {
        return new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kOuttakeProcessorPositionRotations);
    }
    
    public AlgaePivotToPositionCommand createAlgaePivotToBargeOuttakePositionCommand() {
        return new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kOuttakeBargePositionRotations);
    }

    public AlgaePivotToPositionCommand createAlgaePivotToDefualtPositionCommand() {
        return new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kDefultPivotPosition);
    }
}
