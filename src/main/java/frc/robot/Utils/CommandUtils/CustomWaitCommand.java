package frc.robot.Utils.CommandUtils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class CustomWaitCommand extends Command{
    
    private double endTime = -1;
    private double waitTime = 0;
    private String logID = "";

    /**
     * Use this over the WPILib wait command when you need to a wait command to use in a Command Group as there does not work consistently.
     * If you need just a normal wait that does not need to be in command form please use the Wait class as it is more efficent. 
     * @param waitTime Double: The length of the wait in seconds
     */
    public CustomWaitCommand(double waitTime) {
        this.waitTime = waitTime;
    }


    public CustomWaitCommand(double waitTime, String logID) {
        this.waitTime = waitTime;
        Logger.recordOutput(logID + "/WaitCommand", 1);
    }

    @Override
    public void initialize() {
        this.endTime = System.currentTimeMillis() + (waitTime * 1000);//Timer.getFPGATimestamp() + waitTime;
        Logger.recordOutput(logID + "/WaitCommand", 2);

    }

    @Override
    public void execute() {
        Logger.recordOutput(logID + "/WaitCommand", 3);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(logID + "/WaitCommand", 5);
    }

    @Override
    public boolean isFinished() {
        Logger.recordOutput(logID + "/WaitCommand", 4);
        return isScheduled() &&  System.currentTimeMillis() > this.endTime;
    }
}

