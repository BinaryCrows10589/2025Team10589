package frc.robot.Constants.GenericConstants;

import edu.wpi.first.wpilibj.RobotBase;

public class RobotModeConstants {
    public static final boolean isInReplay = false;
    public static final boolean isInSimulation = RobotBase.isSimulation();
    public static final Mode currentMode = 
    isInReplay ? Mode.REPLAY : 
                 (isInSimulation ? Mode.SIM : Mode.REAL);
    public static boolean isBlueAlliance = true;
    public static boolean hasAllianceChanged = false;
    /* Must be set to false before in code before full drive practice and before comp.
     This allows for autons to be proberly ran multiple times with without power cycling
     or redeploying code be reconstructing the robot container when the code is disabled.
    */
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
