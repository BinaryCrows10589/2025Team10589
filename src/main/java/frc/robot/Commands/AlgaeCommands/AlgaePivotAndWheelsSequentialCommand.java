package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeSystem.*;
public class AlgaePivotAndWheelsSequentialCommand extends Command{   
    private AlgaePivotToPositionCommand algaePivotToPositionCommand;
    private RunAlgaeWheelsCommand runAlgaeWheelsCommand;
    private AlgaePivotToPositionCommand algaePivotToDefultCommand;
    private boolean started = false;

    public AlgaePivotAndWheelsSequentialCommand(AlgaePivotToPositionCommand algaePivotToPositionCommand,
        RunAlgaeWheelsCommand runAlgaeWheelsCommand, AlgaePivotToPositionCommand algaePivotToDefultCommand) {
        this.algaePivotToPositionCommand = algaePivotToPositionCommand;
        this.runAlgaeWheelsCommand = runAlgaeWheelsCommand;
        this.algaePivotToDefultCommand = algaePivotToDefultCommand;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.algaePivotToPositionCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(this.algaePivotToPositionCommand.isFinished() && !this.runAlgaeWheelsCommand.isScheduled()) {
            this.runAlgaeWheelsCommand.schedule();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.runAlgaeWheelsCommand.cancel();
        this.algaePivotToPositionCommand.cancel();
        this.algaePivotToDefultCommand.schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
