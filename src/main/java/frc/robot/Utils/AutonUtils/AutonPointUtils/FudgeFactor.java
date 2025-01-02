package frc.robot.Utils.AutonUtils.AutonPointUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FudgeFactor {
    
    private Pose2d blueFudgeFactors;
    private Pose2d redFudgeFactors;

    /**
     * Creates a fudge factor object for the both allience with difforent values.
     * @param xFudgeFactorMetersBlue Double: The x axis fudge factor in meters for blue
     * @param yFudgeFactorMetersBlue Double: The y axis fudge factor in meters for blue
     * @param rotationAngleFudgeFactorDegreesBlue Double: The rotaiton fudge factor in degrees for blue
     * @param xFudgeFactorMetersRed Double: The x axis fudge factor in meters for red
     * @param yFudgeFactorMetersRed Double: The y axis fudge factor in meters for red
     * @param rotationAngleFudgeFactorDegreesRed Double: The rotaiton fudge factor in degrees for for red
     */
    public FudgeFactor(double xFudgeFactorMetersBlue, double yFudgeFactorMetersBlue, double rotationAngleFudgeFactorDegreesBlue,
        double xFudgeFactorMetersRed, double yFudgeFactorMetersRed, double rotationAngleFudgeFactorDegreesRed) {

        this.blueFudgeFactors = new Pose2d(xFudgeFactorMetersBlue, yFudgeFactorMetersBlue, Rotation2d.fromDegrees(rotationAngleFudgeFactorDegreesBlue));
        this.redFudgeFactors = new Pose2d(xFudgeFactorMetersRed, yFudgeFactorMetersRed, Rotation2d.fromDegrees(rotationAngleFudgeFactorDegreesRed));
    }

    /**
     * Creates a fudge factor object for the both alience with difforent values.
     * @param xFudgeFactorMeters Double: The x axis fudge factor in meters for both alliences
     * @param yFudgeFactorMeters Double: The y axis fudge factor in meters for both alliences
     * @param rotationAngleFudgeFactorDegrees Double: The rotaiton fudge factor in degrees for both alliences
     */
    public FudgeFactor(double xFudgeFactorMeters, double yFudgeFactorMeters, double rotationAngleFudgeFactorDegrees) {

        this.blueFudgeFactors = new Pose2d(xFudgeFactorMeters, yFudgeFactorMeters, Rotation2d.fromDegrees(rotationAngleFudgeFactorDegrees));
        this.redFudgeFactors = this.blueFudgeFactors;
    }

    /**
     * Gets the fudge factors for the blue alliance
     * @return Pose2d: The blue alliance fudge factors
     */
    public Pose2d getBlueFudgeFactors() {
        return this.blueFudgeFactors;
    }

    /**
     * Gets the fudge factors for the red alliance
     * @return Pose2d: The red alliance fudge factors
     */
    public Pose2d getRedFudgeFactors() {
        return this.redFudgeFactors;
    }
}
