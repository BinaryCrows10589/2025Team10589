package frc.robot.CrowMotion.Library;

import edu.wpi.first.math.geometry.Translation2d;

public class CMPathGenResult {
    public CMPathPoint[] path;
    public Translation2d[] loggingPoints;
    public int[] rotationDeadlines;
    public CMPathGenResult(CMPathPoint[] path, int[] rotationDeadlines) {
        this.path = path;
        this.rotationDeadlines = rotationDeadlines;
    }
}
