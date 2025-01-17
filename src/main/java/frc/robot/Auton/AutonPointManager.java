package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Utils.AutonUtils.WPILIBTrajectoryConfig;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Constants.FieldConstants;

public class AutonPointManager {

    // Robot start positions
    public static final AutonPoint kOwnAllianceBargeStartPosition =  new AutonPoint(7.15, 5.22, 60, 180, false);
    public static final AutonPoint kCenterBargeStartPosition =  new AutonPoint(7.135, 4.0386, 0, 180, false);
    public static final AutonPoint kOtherAllianceBargeStartPosition = new AutonPoint(7.15, 2.830, 300, 180, false);


    // Deloration of Points

    // Points For Own Alliance
    public static final AutonPoint kPlaceOnCoralB = new AutonPoint(5.250, 5.1518, 60, 180, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralB = new AutonPoint(5.250, 5.1518, 60, 155, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralBConstrant = new AutonPoint(4.5, 5.8, 60, 155, false);
    public static final AutonPoint kIntakeFromOwnAllianceHumanPlayer = new AutonPoint(1.600, 7.25, 126, 180, false);
    public static final AutonPoint kLeaveFromOwnAllianceHumanPlayer = new AutonPoint(1.600, 7.25, 126, 0, false);
    public static final AutonPoint kPlaceOnCoralD = new AutonPoint(3.95, 5.25, 120, 310, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralD = new AutonPoint(3.95, 5.25, 120, 130, false);
    public static final AutonPoint kPlaceOnCoralE = new AutonPoint(3.65, 5.1, 120, 310, false);

    // Points For Center Position
    public static final AutonPoint kPlaceOnCoralL = new AutonPoint(5.82, 3.85, 0, 180, false);
    public static final AutonPoint kPlaceOnCoralA = new AutonPoint(5.82, 4.20, 0, 180, false);
    // Points For Other Alliance 3.0318
    public static final AutonPoint kPlaceOnCoralK = new AutonPoint(5.25, 2.9, -60, 180, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralK = new AutonPoint(5.25, 2.9, -60, -155, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralKConstrant = new AutonPoint(4.5, FieldConstants.kFieldWidthMeters-5.8, -60, -155, false);
    public static final AutonPoint kIntakeFromOtherAllianceHumanPlayer = new AutonPoint(1.600, FieldConstants.kFieldWidthMeters-7.25, -126, -180, false);
    public static final AutonPoint kLeaveFromOtherAllianceHumanPlayer = new AutonPoint(1.600, FieldConstants.kFieldWidthMeters-7.25, -126, -0, false);
    // 2.8018
    public static final AutonPoint kPlaceOnCoralI = new AutonPoint(3.95, FieldConstants.kFieldWidthMeters-5.25, -120, 50, false);
    // 2.6518
    public static final AutonPoint kLeaveFromPlaceOnCoralI = new AutonPoint(3.95, FieldConstants.kFieldWidthMeters-5.25, -120, -130, false);
    public static final AutonPoint kPlaceOnCoralH = new AutonPoint(3.65, FieldConstants.kFieldWidthMeters-5.1, -120, -310, false);
    
    // Decloration of Path Points

    // Path Points for Own Alliance
    public static final AutonPoint[] kOwnAllianceBargeStartPositionToPlaceOnCoralB = {kOwnAllianceBargeStartPosition, kPlaceOnCoralB};
    public static final AutonPoint[] kPlaceOnCoralBToHumanPlayer = {kLeaveFromPlaceOnCoralB, kLeaveFromPlaceOnCoralBConstrant, kIntakeFromOwnAllianceHumanPlayer};
    public static final AutonPoint[] kHumanPlayerToCoralD = {kLeaveFromOwnAllianceHumanPlayer, kPlaceOnCoralD};
    public static final AutonPoint[] kCoralDToHumanPlayer = {kLeaveFromPlaceOnCoralD, kIntakeFromOwnAllianceHumanPlayer};
    public static final AutonPoint[] kHumanPlayerToCoralE = {kLeaveFromOwnAllianceHumanPlayer, kPlaceOnCoralE};
    
    // Path Points for Center Position
    public static final AutonPoint[] kCenterBargeStartPositionToPlaceOnCoralA = {kCenterBargeStartPosition, kPlaceOnCoralA};
    public static final AutonPoint[] kCenterBargeStartPositionToPlaceOnCoralL = {kCenterBargeStartPosition, kPlaceOnCoralL};

    // Path Points for Other Alliance
    public static final AutonPoint[] kOtherAllianceBargeStartPositionToPlaceOnCoralK = {kOtherAllianceBargeStartPosition, kPlaceOnCoralK};
    public static final AutonPoint[] kPlaceOnCoralKToHumanPlayer = {kLeaveFromPlaceOnCoralK, kLeaveFromPlaceOnCoralKConstrant, kIntakeFromOtherAllianceHumanPlayer};
    public static final AutonPoint[] kHumanPlayerToCoralI = {kLeaveFromOtherAllianceHumanPlayer, kPlaceOnCoralI};
    public static final AutonPoint[] kCoralIToHumanPlayer = {kLeaveFromPlaceOnCoralI, kIntakeFromOtherAllianceHumanPlayer};
    public static final AutonPoint[] kHumanPlayerToCoralH = {kLeaveFromOtherAllianceHumanPlayer, kPlaceOnCoralH};
}
