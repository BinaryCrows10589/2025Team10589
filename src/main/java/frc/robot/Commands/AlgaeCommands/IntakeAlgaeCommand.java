package frc.robot.Commands.AlgaeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommands.ElevatorToPositionCommand;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;

public class IntakeAlgaeCommand extends Command {
    
    /*
     * This command needs to
     * - Start moving the elevator to the target position
     * - When the elevator is within a certain distance of the target position, start pivoting the algae to the correct position and run the wheels
     */

    private final ElevatorToPositionCommand elevatorToPositionCommand;
    private final AlgaePivotToPositionCommand pivotCommand;
    private final RunAlgaeWheelsCommand algaeWheelsCommand;

    public IntakeAlgaeCommand(ElevatorToPositionCommand elevatorToPositionCommand, AlgaePivotToPositionCommand pivotCommand, RunAlgaeWheelsCommand algaeWheelsCommand) {
        this.elevatorToPositionCommand = elevatorToPositionCommand;
        this.pivotCommand = pivotCommand;
        this.algaeWheelsCommand = algaeWheelsCommand;
    }

    @Override
    public void initialize() {
        elevatorToPositionCommand.schedule();
    }

    @Override
    public void execute() {
        if (elevatorToPositionCommand.isWithinTolerance(ElevatorConstants.kElevatorIntakeTolerance)) { // If the elevator is close to the right position...
            if (!pivotCommand.isScheduled()) {
                pivotCommand.schedule();
            }
            if (!algaeWheelsCommand.isScheduled()) {
                algaeWheelsCommand.schedule();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorToPositionCommand.cancel();
        pivotCommand.cancel();
        algaeWheelsCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return false; // Don't stop, the button will do that for us.
    }

}
