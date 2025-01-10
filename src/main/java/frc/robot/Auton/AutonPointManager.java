package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Utils.AutonUtils.WPILIBTrajectoryConfig;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class AutonPointManager {

    // Robot start positions
    public static final AutonPoint kOwnAllianceBargeStartPosition =  new AutonPoint(7.99, 5.22, 0, 180, false);

    // Deloration of Points
    public static final AutonPoint kPlaceOnCoralB = new AutonPoint(5.250, 5.020, 60, 180, false);
    public static final AutonPoint kMoveAwayFromReefOwnAlliance = new AutonPoint(4, 5.5, 0, 120, false);
    public static final AutonPoint kIntakeFromOwnAllianceHumanPlayer = new AutonPoint(1.600, 7.35, 36, 180, false);
    // Decloration of Path Points
    public static final AutonPoint[] kOwnAllianceBargeStartPositionToPlaceOnCoralB = {kOwnAllianceBargeStartPosition, kPlaceOnCoralB};
    public static final AutonPoint[] kkPlaceOnCoralBToHumanPlayer = {kPlaceOnCoralB, kMoveAwayFromReefOwnAlliance, kIntakeFromOwnAllianceHumanPlayer};
}
