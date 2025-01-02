package frc.robot.Utils.CommandUtils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class CustomWaitCommand extends Command{
    
    private double endTime = -1;
    private double waitTime = 0;

    /**
     * Use this over the WPILib wait command when you need to a wait command to use in a Command Group as there does not work consistently.
     * If you need just a normal wait that does not need to be in command form please use the Wait class as it is more efficent. 
     * @param waitTime Double: The length of the wait in seconds
     */
    public CustomWaitCommand(double waitTime) {
        this.waitTime = waitTime;
    }

    @Override
    public void initialize() {
        System.out.println("WaitCommand");
        this.endTime = Timer.getFPGATimestamp() + waitTime;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Wiat Command finished");
    }

    @Override
    public boolean isFinished() {
        return isScheduled() && Timer.getFPGATimestamp() > this.endTime;
    }
}

