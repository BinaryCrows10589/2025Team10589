package frc.robot.CrowMotion.Library;

import edu.wpi.first.math.geometry.Translation2d;

public class CMPathGenResult {
    public CMPathPoint[] path;
    public Translation2d[] loggingPoints;

    public CMPathGenResult(CMPathPoint[] path, Translation2d[] loggingPoints) {
        this.path = path;
        this.loggingPoints = loggingPoints;
    }
}
