package frc.robot.Utils.OdometryUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotModeConstants;
import frc.robot.Utils.AutonUtils.AutonPointUtils.FudgeFactor;

public class FudgedPoint {
    
    private Pose2d fudgedPoint;
    private FudgeFactor fudgeFactor;
    
    /**
     * @param xPointMeters The x coordnet in meters
     * @param yPointMeters The y coordnet in meters
     * @param rotationAngleDegrees The rotation angle in degrees
     * @param fudgeFactor
     */
    public FudgedPoint(double xPointMeters, double yPointMeters, double rotationAngleDegrees, FudgeFactor fudgeFactor) {
        this.fudgedPoint = new Pose2d(xPointMeters, yPointMeters, Rotation2d.fromDegrees(rotationAngleDegrees));
        this.fudgeFactor = fudgeFactor;
    }

    /**
     * @param pose Pose2d: The x, y and rotaiton value of the point
     * @param fudgeFactor
     */
    public FudgedPoint(Pose2d pose, FudgeFactor fudgeFactor) {
        this.fudgedPoint = pose;
        this.fudgeFactor = fudgeFactor;
    }

    /**
     * @param xPointMeters The x coordnet in meters
     * @param yPointMeters The y coordnet in meters
     * @param rotationAngleDegrees The rotation angle in degrees
     */
    public FudgedPoint(double xPointMeters, double yPointMeters, double rotationAngleDegrees) {
        this.fudgedPoint = new Pose2d(xPointMeters, yPointMeters, Rotation2d.fromDegrees(rotationAngleDegrees));
        this.fudgeFactor = new FudgeFactor(0, 0, 0);
    }

    /**
     * Gets the auton point. Auto mirred depending on alliance.
     * @return Pose2d: The auton point which is mirred for the current alliance.
     */
    public Pose2d getFudgedPoint() {
        return new Pose2d(this.fudgedPoint.getX() + this.fudgeFactor.getBlueFudgeFactors().getX(),
            this.fudgedPoint.getY() + this.fudgeFactor.getBlueFudgeFactors().getY(),
                Rotation2d.fromDegrees((this.fudgedPoint.getRotation().getDegrees() +
                this.fudgeFactor.getBlueFudgeFactors().getRotation().getDegrees())));
    } 
}
