package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotSubsystem;

public class PivotToPositionCommand extends Command {
    private final PivotSubsystem pivotSubsystem;
    private final double targetPosition;
    

    public PivotToPositionCommand(double targetPosition, PivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(this.pivotSubsystem);
    }

    @Override
    public void initialize() {
        this.pivotSubsystem.setDesiredPivotRotation(targetPosition);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return this.pivotSubsystem.isPivotInTolorence();
    }
}
