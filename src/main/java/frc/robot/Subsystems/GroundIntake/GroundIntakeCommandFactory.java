package frc.robot.Subsystems.GroundIntake;

import frc.robot.Commands.IntakeCommands.PivotToPositionCommand;
import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.PivotContants;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotSubsystem;

public class GroundIntakeCommandFactory {

    private final PivotSubsystem pivotSubsystem;

    public GroundIntakeCommandFactory(PivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
    }
     
    public PivotToPositionCommand createPivotDownCommand() {
        return new PivotToPositionCommand(PivotContants.kPivotDownPosition, pivotSubsystem);
    }

    public PivotToPositionCommand createPivotUpCommand() {
        return new PivotToPositionCommand(PivotContants.kPivotUpPosition, pivotSubsystem);
    }
}
