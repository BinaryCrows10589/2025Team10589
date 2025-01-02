package frc.robot.Utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.Command;

public class SequentialGroupCommand extends Command {
    private Command[] commands;
    private int currentRunningIndex = -1;

    public SequentialGroupCommand(Command[] commands) {
        this.commands = commands;
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
        
        if (this.commands[currentRunningIndex].isFinished()) {
            currentRunningIndex++;
            if (currentRunningIndex >= this.commands.length) {
                return;
            }
            this.commands[currentRunningIndex].schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return currentRunningIndex >= this.commands.length;
    }
}
