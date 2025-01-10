package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Utils.AutonUtils.WPILIBTrajectoryConfig;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class AutonPointManager {

    // Keep the first three argumetns of auton point being the goal robot state, have a way to set a angle of attack for every point
    // Robot start positions
    public static final AutonPoint kOwnAllianceBargeStartPosition =  new AutonPoint(7.99, 5.22, 0);

    // Deloration of Points
    public static final AutonPoint kPlaceOnCoralB = new AutonPoint(5.250, 5.020, 0);

    // Decloration of Path Points
    public static final AutonPoint[] kOwnAllianceBargeStartPositionToPlaceOnCoralB = {kOwnAllianceBargeStartPosition, kPlaceOnCoralB};
}
