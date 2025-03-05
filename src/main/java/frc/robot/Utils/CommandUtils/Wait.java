package frc.robot.Utils.CommandUtils;
import edu.wpi.first.wpilibj.Timer;

public class Wait {
    private double endTime = Double.MAX_VALUE;
    private double waitTime = 0;
    private boolean timerEnabled = true;
    /**
     *  Use this when you need a wait that is not in a group command. It is much more efficent that the CustomWaitCommand
     * @param waitTime Double: The length of the wait in seconds
     */
    public Wait(double waitTime) {
        this.waitTime = waitTime;
    }

    private long getAccurateSystemTime() {
        return System.currentTimeMillis();
    }

    /**
     *  This must be called to start the timmer
     */
    public void startTimer() {
        this.endTime = getAccurateSystemTime() + (waitTime * 1000);
        this.timerEnabled = true;
    }

    /**
     *  Returns whether or not the time has passed
     * @return Boolean: Whether or not the desired wait time has passed. 
     */
    public boolean hasTimePassed() {
        return getAccurateSystemTime() > this.endTime && this.timerEnabled;
    }

    public void disableTimer() {
        this.timerEnabled = false;
    }
}
