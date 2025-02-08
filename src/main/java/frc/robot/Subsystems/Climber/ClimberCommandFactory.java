package frc.robot.Subsystems.Climber;

import frc.robot.Commands.ClimberCommands.MoveClimberManuallyCommand;
import frc.robot.Constants.MechanismConstants.ClimberConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;

public class ClimberCommandFactory {
    private final ClimberSubsystem climberSubsystem;


    public ClimberCommandFactory(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }

    public MoveClimberManuallyCommand createMoveClimberUpManuallyCommand() {
        return new MoveClimberManuallyCommand(ClimberConstants.kClimberUpSpeed, climberSubsystem);
    }
    public MoveClimberManuallyCommand createMoveClimberDownManuallyCommand() {
        return new MoveClimberManuallyCommand(ClimberConstants.kClimberDownSpeed, climberSubsystem);
    }
}
