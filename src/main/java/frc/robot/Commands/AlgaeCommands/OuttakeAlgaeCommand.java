package frc.robot.Commands.AlgaeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommands.ElevatorToPositionCommand;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Utils.LEDUtils.LEDManager;

public class OuttakeAlgaeCommand extends Command {

    /*
     * This command needs to
     * - Start moving the elevator to the target position
     * - When the elevator is within a certain distance of the target position, start pivoting the algae to the correct position
     * - When the elevator command is complete, wait for the output button to be pressed (boolean supplier) and then spit out the algae
     */

    private final BooleanSupplier algaeOuttakeButton;
    private final ElevatorToPositionCommand elevatorToPositionCommand;
    private final AlgaePivotToPositionCommand pivotCommand;
    private final RunAlgaeWheelsCommand algaeWheelsCommand;

    public OuttakeAlgaeCommand(BooleanSupplier algaeOuttakeButton, ElevatorToPositionCommand elevatorToPositionCommand, AlgaePivotToPositionCommand pivotCommand, RunAlgaeWheelsCommand algaeWheelsCommand) {
        this.algaeOuttakeButton = algaeOuttakeButton;
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
        if (elevatorToPositionCommand.isWithinTolerance(ElevatorConstants.kElevatorOuttakeTolerance)) { // If the elevator is close to the right position...
            if (pivotCommand.isFinished()) { // If our pivot is in the right spot...
                LEDManager.setSolidColor(ControlConstants.kAlgaeOuttakeReady);
                if (algaeOuttakeButton.getAsBoolean()) { // If we are pressing the outtake button...
                    // If the algae wheel command isn't already scheduled, schedule it
                    if (!algaeWheelsCommand.isScheduled()) {
                        algaeWheelsCommand.schedule();
                    }
                }
            } else {
                // If the pivot command isn't already scheduled, schedule it
                if (!pivotCommand.isScheduled()) {
                    pivotCommand.schedule();
                }
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
