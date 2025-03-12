package frc.robot.Subsystems.Climber;

import frc.robot.Commands.ClimberCommands.RunClimberAtVoltageCommand;
import frc.robot.Constants.MechanismConstants.ClimberConstants;

public class ClimberCommandFactory {
    
    public ClimberSubsystem climberSubsystem;

    public ClimberCommandFactory(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }

    public RunClimberAtVoltageCommand createpPivotClimberDownCommand() {
        return new RunClimberAtVoltageCommand(this.climberSubsystem, ClimberConstants.kClimberDownVoltage);
    }

    public RunClimberAtVoltageCommand createPivotClimberUpCommnad() {
        return new RunClimberAtVoltageCommand(this.climberSubsystem, ClimberConstants.kClimberUpVoltage);
    }
}
