package frc.robot.Commands.ClimberCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber.ClimberSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem.ElevatorMode;

public class MoveClimberManuallyCommand extends Command {

    private final ClimberSubsystem climberSubsystem;
    private final double climberSpeed;

    public MoveClimberManuallyCommand(double climberSpeed, ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.climberSpeed = climberSpeed;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climberSubsystem.incrementDesiredClimberPosition(climberSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
