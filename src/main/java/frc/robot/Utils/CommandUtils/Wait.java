package frc.robot.Utils.CommandUtils;
import edu.wpi.first.wpilibj.Timer;

public class Wait {
    private double endTime = Double.MAX_VALUE;
    private double waitTime = 0;

    /**
     *  Use this when you need a wait that is not in a group command. It is much more efficent that the CustomWaitCommand
     * @param waitTime Double: The length of the wait in seconds
     */
    public Wait(double waitTime) {
        this.waitTime = waitTime;
    }

    /**
     *  This must be called to start the timmer
     */
    public void startTimer() {
        this.endTime = Timer.getFPGATimestamp() + waitTime;
    }

    /**
     *  Returns whether or not the time has passed
     * @return Boolean: Whether or not the desired wait time has passed. 
     */
    public boolean hasTimePassed() {
        return Timer.getFPGATimestamp() > this.endTime;
    }
}
