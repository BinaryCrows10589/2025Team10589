package frc.robot.Utils.CommandUtils;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.wpilibj2.command.Command;

public class SequentialGroupCommand extends Command {
    private Command[] commands;
    private int currentRunningIndex = -1;
    private CustomWaitCommand waitCommand;

    public SequentialGroupCommand(Command... commands) {
        this.commands = commands;
        this.waitCommand = new CustomWaitCommand(100000);
    }

    public SequentialGroupCommand(double waitTime, Command... commands) {
        this.commands = commands;
        this.waitCommand = new CustomWaitCommand(waitTime);
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
        }

        if(!(this.commands[currentRunningIndex] instanceof SequentialGroupCommand) && !this.waitCommand.isScheduled()) {
            this.waitCommand.schedule();
        }
        
        if (this.commands[currentRunningIndex].isFinished() || this.waitCommand.isFinished()) {
            if(this.waitCommand.isFinished()) {
                this.commands[currentRunningIndex].cancel();
            }
            this.waitCommand.cancel();
            currentRunningIndex++;
            
            if (currentRunningIndex >= this.commands.length) {
                return;
            }
            this.commands[currentRunningIndex].schedule();
        }
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
