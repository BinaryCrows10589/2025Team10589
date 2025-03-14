package frc.robot.Commands.ElevatorCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Utils.CommandUtils.Wait;

public class LiftAfterTimeWhenCoralIsInCommand extends Command{

    private Wait waitForElevator;
    private ElevatorToPositionCommand elevatorToPositionCommand;
    public LiftAfterTimeWhenCoralIsInCommand(ElevatorToPositionCommand elevatorToPositionCommand, double waitTime) {
        this.waitForElevator = new Wait(waitTime);
        this.elevatorToPositionCommand = elevatorToPositionCommand;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  
        this.waitForElevator.startTimer();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Logger.recordOutput("LiftAfterTimeAndHasCoral", this.waitForElevator.hasTimePassed() && ControlConstants.kHasCoral && !this.elevatorToPositionCommand.isScheduled());
        if(this.waitForElevator.hasTimePassed() && ControlConstants.kHasCoral && !this.elevatorToPositionCommand.isScheduled()) {
            this.elevatorToPositionCommand.schedule();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.waitForElevator.disableTimer();
        this.elevatorToPositionCommand.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.elevatorToPositionCommand.isFinished() && this.elevatorToPositionCommand.isScheduled();
    }
}
