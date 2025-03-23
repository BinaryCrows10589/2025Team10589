package frc.robot.Utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;

public class SequentialGroupCommand extends Command {
    private Command[] commands;
    private int currentRunningIndex = -1;
    private CustomWaitCommand waitCommand;
    private double waitTime;
    private double trajectoryWaitTime;
    public boolean hasTrajectory = false;

    public SequentialGroupCommand(Command... commands) {
        this.commands = commands;
        this.waitCommand = new CustomWaitCommand(100000);
    }

    
    public SequentialGroupCommand(double waitTime, double trajectoryWaitTime, Command... commands) {
        this.commands = commands;
        this.waitTime = waitTime;
        this.trajectoryWaitTime = trajectoryWaitTime;
        this.waitCommand = new CustomWaitCommand(waitTime);
    }
   

    @Override
    public void initialize() {
        for(int i = 0; i < commands.length; i++) {
            if(this.commands[i] instanceof WPILibFollowTrajectoryFromPointsCommand) {
                hasTrajectory = true;
            }
        }
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

        if(!this.waitCommand.isScheduled()) {
            if(this.commands[currentRunningIndex] instanceof WPILibFollowTrajectoryFromPointsCommand || 
                this.commands[currentRunningIndex] instanceof ParallelGroupCommand || hasTrajectory) {
                    this.waitCommand = new CustomWaitCommand(trajectoryWaitTime);
                    this.waitCommand.schedule();
            } else if(!(this.commands[currentRunningIndex] instanceof SequentialGroupCommand)) {
                this.waitCommand = new CustomWaitCommand(waitTime);
                this.waitCommand.schedule();
            }
        }

        // OLD
        /* 
        if(!(this.commands[currentRunningIndex] instanceof SequentialGroupCommand ||
         this.commands[currentRunningIndex] instanceof WPILibFollowTrajectoryFromPointsCommand ||
         this.commands[currentRunningIndex] instanceof ParallelGroupCommand) && !this.waitCommand.isScheduled()) {
            this.waitCommand.schedule();
        }*/
        
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
