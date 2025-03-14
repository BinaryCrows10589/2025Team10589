package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;

public class MoveElevatorManuallyCommand extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final double elevatorSpeed;

    public MoveElevatorManuallyCommand(double elevatorSpeed, ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorSpeed = elevatorSpeed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevatorSubsystem.incrementDesiredElevatorPosition(elevatorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
