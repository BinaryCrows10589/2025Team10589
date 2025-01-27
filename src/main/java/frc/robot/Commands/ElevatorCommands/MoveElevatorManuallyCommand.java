package frc.robot.Commands.ElevatorCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem.ElevatorMode;

public class MoveElevatorManuallyCommand extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final double elevatorSpeed;
    private final Supplier<Boolean> isRunning;
    
    public MoveElevatorManuallyCommand(double elevatorSpeed, Supplier<Boolean> isRunning, ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorSpeed = elevatorSpeed;
        this.isRunning = isRunning;
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
        return isRunning.get();
    }
}
