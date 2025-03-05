package frc.robot.Utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.Command;

public class SequentialGroupCommand extends Command {
    private Command[] commands;
    private int currentRunningIndex = -1;

    private Wait maxTimePerCommand = null;

    public SequentialGroupCommand(Command... commands) {
        this.commands = commands;
    }

    public SequentialGroupCommand(int maxWaitTime, Command... commands) {
        this(commands);
        maxTimePerCommand = new Wait(maxWaitTime);
    }
   

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (currentRunningIndex >= this.commands.length) {
                return;
        }

        if (currentRunningIndex == -1) {
            currentRunningIndex = 0;
            this.commands[currentRunningIndex].schedule();
            resetMaxCommandTimer();
        }
        
        if (this.commands[currentRunningIndex].isFinished() || maxCommandTimePassed()) {

            // BOYNE: Possible fix? I don't know to be honest
            if (this.commands[currentRunningIndex].isScheduled()) {
                this.commands[currentRunningIndex].cancel();
            }

            currentRunningIndex++;
            if (currentRunningIndex >= this.commands.length) {
                return;
            }
            
            this.commands[currentRunningIndex].schedule();
            resetMaxCommandTimer();
        }

        
    }

    private boolean maxCommandTimePassed() {
        return maxTimePerCommand != null && maxTimePerCommand.hasTimePassed();
    }
    private void resetMaxCommandTimer() {
        if (maxTimePerCommand != null) maxTimePerCommand.startTimer();
    }

    @Override
    public void end(boolean interrupted) {
        for(Command command : commands) {
            command.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return currentRunningIndex >= this.commands.length;
    }
}
