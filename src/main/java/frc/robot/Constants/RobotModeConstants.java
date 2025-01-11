package frc.robot.Constants;

public class RobotModeConstants {
    public static final Mode currentMode = Mode.REAL;
    public static boolean isBlueAlliance = true;
    public static boolean hasAllianceChanged = false;
    /* Must be set to false before in code before full drive practice and before comp.
     This allows for autons to be proberly ran multiple times with without power cycling
     or redeploying code be reconstructing the robot container when the code is disabled.
    */
    //TODO: Fix this to make it work
    public static boolean kAutonDebugMode = false;
    public static final boolean kIsNotSim = currentMode != Mode.SIM;
    public static final double kLoopPeriod = .02; 


    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY,
    }
}
