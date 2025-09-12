package frc.robot.CrowMotion.Library;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CrowMotion.UserSide.CMEvent;
import frc.robot.CrowMotion.UserSide.CMRotation;

public class CMPathGenResult {
    public CMPathPoint[] path;
    public CMRotation[] rotationDeadlines;
    public CMEvent[] events;

    public CMPathGenResult(CMPathPoint[] path, CMRotation[] rotationDeadlines,
        CMEvent[] events) {
            
        this.path = path;
        this.rotationDeadlines = rotationDeadlines;
        this.events = events;
    }
}
