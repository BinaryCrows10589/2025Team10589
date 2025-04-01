package frc.robot.Utils.CommandUtils;

import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.Command;

/*
A simple command that will stop another command when a supplier returns true.
*/
public class EndCommandUponCondition extends Command {

    private Command command;
    private Supplier<Boolean> conditionalSupplier;
    private boolean hasInitialized = false;

    public EndCommandUponCondition(Command command, Supplier<Boolean> conditionalSupplier) {
        this.command = command;
        this.conditionalSupplier = conditionalSupplier;
    }

    @Override
    public void initialize() {
        this.command.schedule();
        hasInitialized = true;
    }

    @Override
    public void execute() {
        if (this.command.isScheduled() && this.conditionalSupplier.get()) {
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
