package frc.robot.Commands.AlgaeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommands.ElevatorToPositionCommand;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.MechanismConstants.AlgaePivotConstants;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Utils.LEDUtils.LEDManager;

public class IntakeAlgaeCommand extends Command {

    /*
     * This command needs to
     * - Start moving the elevator to the target position
     * - When the elevator is within a certain distance of the target position, start pivoting the algae to the correct position
     * - When the elevator command is complete, wait for the output button to be pressed (boolean supplier) and then spit out the algae
     */

    private final ElevatorToPositionCommand elevatorToPositionCommand;
    private final ElevatorToPositionCommand elevatorToBasementCommand;
    private final AlgaePivotToPositionCommand pivotCommand;
    private final AlgaePivotToPositionCommand defualtPivotPositionCommand;
    private final RunAlgaeWheelsCommand algaeWheelsCommand;

    public IntakeAlgaeCommand(ElevatorToPositionCommand elevatorToPositionCommand,
        AlgaePivotToPositionCommand pivotCommand,
        RunAlgaeWheelsCommand algaeWheelsCommand,
        ElevatorCommandFactory elevatorCommandFactory,
        AlgaePivotSubsystem algaePivotSubsystem) {
        this.elevatorToPositionCommand = elevatorToPositionCommand;
        this.elevatorToBasementCommand = elevatorCommandFactory.createElevatorToBasementCommand();
        this.defualtPivotPositionCommand = new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kDefultPivotPosition);
        this.pivotCommand = pivotCommand;
        this.algaeWheelsCommand = algaeWheelsCommand;
    }

    @Override
    public void initialize() {
        this.elevatorToPositionCommand.schedule();
        this.pivotCommand.schedule();
        this.algaeWheelsCommand.schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        this.elevatorToBasementCommand.schedule();
        this.defualtPivotPositionCommand.schedule();
        this.algaeWheelsCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return false; // Don't stop, the button will do that for us.
    }
    
}
