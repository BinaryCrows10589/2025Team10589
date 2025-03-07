package frc.robot.Utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.Command;

/*
A simple command to replace the WPILib ParallelRaceCommandGroup that doesn't work.
It only works with two commands as of right now. 
*/
public class ParallelGroupCommand extends Command {
    private Command[] command;
    private CustomWaitCommand waitCommand;


    public ParallelGroupCommand(Command... command) {
        this.command = command;
        this.waitCommand = new CustomWaitCommand(100000);
    }
    public ParallelGroupCommand(double waitTime, Command... command) {
        this.command = command;
        this.waitCommand = new CustomWaitCommand(waitTime);
    }

    @Override
    public void initialize() {
        for(int i = 0; i < this.command.length; i++) {
            this.command[i].schedule();
        }

    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished() {
        for(int i = 0; i < this.command.length; i++) {
             if(!this.command[i].isFinished()) {
                return false;
             }
        }
        return true;
    }
}
