package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Utils.AutonUtils.WPILIBTrajectoryConfig;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class AutonPointManager {

    // Robot start positions
    public static final AutonPoint kOwnAllianceBargeStartPosition =  new AutonPoint(7.15, 5.22, 60, 180, false);

    // Deloration of Points
    public static final AutonPoint kPlaceOnCoralB = new AutonPoint(5.250, 5.020, 60, 180, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralB = new AutonPoint(5.250, 5.020, 60, 155, false);
    public static final AutonPoint kIntakeFromOwnAllianceHumanPlayer = new AutonPoint(1.600, 7.35, 36, 180, false);
    public static final AutonPoint kLeaveFromOwnAllianceHumanPlayer = new AutonPoint(1.600, 7.35, 36, 0, false);
    public static final AutonPoint kPlaceOnCoralD = new AutonPoint(3.95, 5.23, 120, 310, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralD = new AutonPoint(3.95, 5.23, 120, 130, false);
    public static final AutonPoint kPlaceOnCoralE = new AutonPoint(3.65, 5.07, 120, 310, false);
    // Decloration of Path Points
    public static final AutonPoint[] kOwnAllianceBargeStartPositionToPlaceOnCoralB = {kOwnAllianceBargeStartPosition, kPlaceOnCoralB};
    public static final AutonPoint[] kPlaceOnCoralBToHumanPlayer = {kLeaveFromPlaceOnCoralB, kIntakeFromOwnAllianceHumanPlayer};
    public static final AutonPoint[] kHumanPlayerToCoralD = {kLeaveFromOwnAllianceHumanPlayer, kPlaceOnCoralD};
    public static final AutonPoint[] kCoralDToHumanPlayer = {kLeaveFromPlaceOnCoralD, kIntakeFromOwnAllianceHumanPlayer};
    public static final AutonPoint[] kHumanPlayerToCoralE = {kLeaveFromOwnAllianceHumanPlayer, kPlaceOnCoralE};
    
}
