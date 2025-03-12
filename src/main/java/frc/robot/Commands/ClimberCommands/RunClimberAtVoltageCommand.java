package frc.robot.Commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber.ClimberSubsystem;

public class RunClimberAtVoltageCommand extends Command{
    
    private final ClimberSubsystem climberSubsystem;
    private final double desiredVoltage;

    public RunClimberAtVoltageCommand(ClimberSubsystem climberSubsystem, double desiredVoltage) {
        this.climberSubsystem = climberSubsystem;
        this.desiredVoltage = desiredVoltage;
    }

    @Override
    public void initialize() {
        this.climberSubsystem.setClimberVoltage(desiredVoltage);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted) {
        this.climberSubsystem.setClimberVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
