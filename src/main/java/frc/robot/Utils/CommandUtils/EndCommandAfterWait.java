package frc.robot.Utils.CommandUtils;

import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.Command;

/*
A simple command that will stop another command when a supplier returns true.
*/
public class EndCommandAfterWait extends Command {

    private Command command;
    private Wait timer;
    private boolean hasInitialized = false;

    public EndCommandAfterWait(Command command, double waitTime) {
        this.command = command;
        this.timer = new Wait(waitTime);
    }

    @Override
    public void initialize() {
        this.command.schedule();
        timer.startTimer();
        hasInitialized = true;
    }

    @Override
    public void execute() {
        if (this.command.isScheduled() && this.timer.hasTimePassed()) {
            this.command.cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (!hasInitialized) return false;
        return !this.command.isScheduled();
    }
}
