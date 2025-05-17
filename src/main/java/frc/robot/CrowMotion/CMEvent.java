package frc.robot.CrowMotion;

public class CMEvent {

    private String eventName;
    private Runnable eventFunction;
    private double eventTriggerPercent;

    /**
     * Constructs a new CMEvent with the specified event name, event function, and trigger percentage.
     *
     * @param eventName The name of the event, used for logging
     * @param eventFunction The function to be executed when the event is triggered.
     * @param eventTriggerPercent The percent along a path at which the event will be triggered. 
     */
    public CMEvent(String eventName, Runnable eventFunction, double eventTriggerPercent) {
        this.eventName = eventName;
        this.eventFunction = eventFunction;
        this.eventTriggerPercent = eventTriggerPercent;
    }

    /**
     * Gets the name of the event.
     *
     * @return The event name.
     */
    public String getEventName() {
        return eventName;
    }

    /**
     * Gets the function that will be executed when the event is triggered.
     *
     * @return The event function (Runnable).
     */
    public Runnable getEventFunction() {
        return eventFunction;
    }

    /**
     * Gets the percent along a path at which the event will be triggered. 
     *
     * @return The percent along a path at which the event will be triggered. 
     */
    public double getEventTriggerPercent() {
        return eventTriggerPercent;
    }

}