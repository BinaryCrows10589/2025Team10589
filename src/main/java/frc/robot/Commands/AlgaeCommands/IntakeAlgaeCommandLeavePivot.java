package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommands.ElevatorToPositionCommand;
import frc.robot.Constants.MechanismConstants.AlgaePivotConstants;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotSubsystem;
import frc.robot.Subsystems.AlgaeSystem.AlgaeWheels.AlgaeWheelSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;

public class IntakeAlgaeCommandLeavePivot extends Command {

    /*
     * This command needs to
     * - Start moving the elevator to the target position
     * - When the elevator is within a certain distance of the target position, start pivoting the algae to the correct position
     * - When the elevator command is complete, wait for the output button to be pressed (boolean supplier) and then spit out the algae
     */

    private final ElevatorToPositionCommand elevatorToPositionCommand;
    private final AlgaePivotToPositionCommand pivotCommand;
    private final AlgaePivotToPositionCommand defualtPivotPositionCommand;
    private final RunAlgaeWheelsCommand algaeWheelsCommand;

    private final AlgaeWheelSubsystem algaeWheelSubsystem;

    public IntakeAlgaeCommandLeavePivot(ElevatorToPositionCommand elevatorToPositionCommand,
        AlgaePivotToPositionCommand pivotCommand,
        RunAlgaeWheelsCommand algaeWheelsCommand,
        ElevatorCommandFactory elevatorCommandFactory,
        AlgaePivotSubsystem algaePivotSubsystem,
        AlgaeWheelSubsystem algaeWheelSubsystem) {
        this.elevatorToPositionCommand = elevatorToPositionCommand;
        this.defualtPivotPositionCommand = new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kDefultPivotPosition);
        this.pivotCommand = pivotCommand;
        this.algaeWheelsCommand = algaeWheelsCommand;
        this.algaeWheelSubsystem = algaeWheelSubsystem;

        addRequirements(algaeWheelSubsystem);
    }

    @Override
    public void initialize() {
        this.elevatorToPositionCommand.schedule();
        this.pivotCommand.schedule();
        this.algaeWheelsCommand.schedule();

        algaeWheelSubsystem.setAlgaeWheelPulse(true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        this.algaeWheelsCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return false; // Don't stop, the button will do that for us.
    }
    
}
