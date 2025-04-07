package frc.robot.Auton;

import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.AutonUtils.AutonPointUtils.FudgeFactor;

public class AutonPointManager {

    // Robot start positions
    public static final AutonPoint kOwnAllianceBargeStartPosition =  new AutonPoint(7.15, 5.2218, 60, 180, false);
    public static final AutonPoint kCenterBargeStartPosition =  new AutonPoint(7.15, 4.0386, 0, 180, false);
    public static final AutonPoint kOtherAllianceBargeStartPosition = new AutonPoint(7.15, 2.830, 300, 180, false);


    // Deloration of Points

    // Points For Own Alliance
    public static final AutonPoint kPlaceOnCoralB = new AutonPoint(5.27, 5.05, 59, 180, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralB = new AutonPoint(5.270, 5.1818, 60, 135, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralBConstrant = new AutonPoint(5.2, 5.3518, 60, 120, false);
    public static final AutonPoint kIntakeFromOwnAllianceHumanPlayer = new AutonPoint(1.25, 6.9018, 100, 180, false);
    public static final AutonPoint kLeaveFromOwnAllianceHumanPlayer = new AutonPoint(1.25, 6.9018, 100, -60, false);
    public static final AutonPoint kPlaceOnCoralD = new AutonPoint(3.84, 5.09, 125, 50, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralD = new AutonPoint(3.85, 4.9818, 120, 130, false);
    public static final AutonPoint kPlaceOnCoralE = new AutonPoint(3.76, 5.3018, 115, 310, false);

    // Points For Center Position
    public static final AutonPoint kPlaceOnCoralL = new AutonPoint(5.8, 4.06, 0, 180, false);
    public static final AutonPoint kLeaveFromPlaceOnCoralL = new AutonPoint(5.82, 3.85, 0, 0, false);

    public static final AutonPoint kPlaceOnCoralA = new AutonPoint(5.8, 4.20,0, 180, false);

    public static final AutonPoint kBackUpBeforeAlgaeIntake = new AutonPoint(6.5, 4.06, 0, 0, false);
    public static final AutonPoint kLeaveFromBackUpBeforeAlgaeIntake = new AutonPoint(7, 4.06, 0, 180, false);
    public static final AutonPoint kIntakeCenterAlgae = new AutonPoint(5.82, 4.06, 0, 180, false);
    public static final AutonPoint kLeaveFromIntakeCenterAlgae = new AutonPoint(5.82, 4.06, 0, 0, false);
    public static final AutonPoint kBackUpFromAlgaeIntake = new AutonPoint(6.5, 4.061, 0, 180, false);
    public static final AutonPoint kPlaceAlgaeNetStopPoint = new AutonPoint(7.75, 5.5, 180, 90, false);
    public static final AutonPoint kPlaceAlgaeNet = new AutonPoint(8, 5.5, 180, 0, false);

    // Points For Other Alliance 3.0318
    public static final AutonPoint kPlaceOnCoralK = new AutonPoint(5.04, 2.88, -61.0, 180, false, new FudgeFactor(0, 0, 0, .03, .045, 0));
    public static final AutonPoint kLeaveFromPlaceOnCoralK = new AutonPoint(5.27, 2.87, -60, -135, false, new FudgeFactor(0, 0, 0, 0, 0, 0));
    public static final AutonPoint kLeaveFromPlaceOnCoralKConstrant = new AutonPoint(5.2, 2.7, -60, -120, false, new FudgeFactor(0, 0, 0, 0, 0, 0));
    public static final AutonPoint kIntakeFromOtherAllianceHumanPlayer = new AutonPoint(1.2, 1.15, -115, -180, false, new FudgeFactor(0, 0, 0, 0, 0, 0));
    public static final AutonPoint kLeaveFromOtherAllianceHumanPlayer = new AutonPoint(1.2, 1.15, -115, 60, false, new FudgeFactor(0, 0, 0, 0, 0, 0));
    public static final AutonPoint kPlaceOnCoralI = new AutonPoint(3.84, 2.8725, -115, 50, false, new FudgeFactor(0, 0, 0, 0, 0, 0));
    public static final AutonPoint kLeaveFromPlaceOnCoralI = new AutonPoint(3.85, 3.07, -120, -130, false, new FudgeFactor(0, 0, 0, 0, 0, 0));
    public static final AutonPoint kPlaceOnCoralH = new AutonPoint(3.76, 2.75, -125, -310, false, new FudgeFactor(0, 0, -3, 0.05, .15, -3));
    //-200
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
    public static final AutonPoint[] kPlaceCoralLToBackUpBeforeIntakeAlgae = {kLeaveFromPlaceOnCoralL, kBackUpBeforeAlgaeIntake};
    public static final AutonPoint[] kCenterBargeStartPositionToIntakeCenterAlgae = {kLeaveFromBackUpBeforeAlgaeIntake, kIntakeCenterAlgae};
    public static final AutonPoint[] kCenterBargeStartPositionBackUpFromIntakeCenterAlgae = {kLeaveFromIntakeCenterAlgae, kBackUpFromAlgaeIntake};
    public static final AutonPoint[] kCenterBargeStartPositionPlaceAlgaePosition = {kBackUpFromAlgaeIntake, kPlaceAlgaeNetStopPoint};
    public static final AutonPoint[] kCenterBargeStartPositionPlaceSlowDriveInAlgaePosition = {kPlaceAlgaeNetStopPoint, kPlaceAlgaeNet};
    

    // Path Points for Other Alliance
    public static final AutonPoint[] kOtherAllianceBargeStartPositionToPlaceOnCoralK = {kOtherAllianceBargeStartPosition, kPlaceOnCoralK};
    public static final AutonPoint[] kPlaceOnCoralKToHumanPlayer = {kLeaveFromPlaceOnCoralK, kLeaveFromPlaceOnCoralKConstrant, kIntakeFromOtherAllianceHumanPlayer};
    public static final AutonPoint[] kHumanPlayerToCoralI = {kLeaveFromOtherAllianceHumanPlayer, kPlaceOnCoralI};
    public static final AutonPoint[] kCoralIToHumanPlayer = {kLeaveFromPlaceOnCoralI, kIntakeFromOtherAllianceHumanPlayer};
    public static final AutonPoint[] kHumanPlayerToCoralH = {kLeaveFromOtherAllianceHumanPlayer, kPlaceOnCoralH};
}
