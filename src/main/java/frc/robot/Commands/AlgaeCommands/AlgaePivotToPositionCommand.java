package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotSubsystem;

public class AlgaePivotToPositionCommand extends Command {
    
    private final AlgaePivotSubsystem algaePivotSubsystem;
    private final double targetPosition;

    public AlgaePivotToPositionCommand(AlgaePivotSubsystem algaePivotSubsystem, double targetPosition) {
        this.algaePivotSubsystem = algaePivotSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(this.algaePivotSubsystem);
    }

    @Override
    public void initialize() {
        this.algaePivotSubsystem.setDesiredPivotRotation(targetPosition);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return this.algaePivotSubsystem.isPivotInTolerance();
    }



}
