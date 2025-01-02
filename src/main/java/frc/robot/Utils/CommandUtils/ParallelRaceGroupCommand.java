package frc.robot.Utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.Command;

/*
A simple command to replace the WPILib ParallelRaceCommandGroup that doesn't work.
It only works with two commands as of right now. 
*/
public class ParallelRaceGroupCommand extends Command {
    private Command[] commands;
    private int indexedOfFirstFinishedCommand = -1;

    public ParallelRaceGroupCommand(Command... command) {
        this.commands = command;
    }

    @Override
    public void initialize() {
        for(int i = 0; i < commands.length; i++) {
            this.commands[i].schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        for(int i = 0; i < this.commands.length && i != indexedOfFirstFinishedCommand; i++) {
            this.commands[i].cancel();
        }
    }

    @Override
    public boolean isFinished() {
        for(int i = 0; i < commands.length; i++) {
            if(this.commands[i].isFinished()) {
                this.indexedOfFirstFinishedCommand = i;
                return true;
            }
        }
        return false;
    }
}
