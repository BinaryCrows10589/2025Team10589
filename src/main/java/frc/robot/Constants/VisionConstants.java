package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.AutonUtils.AutonPointUtils.FudgeFactor;

// Do not create an instants of a constant class

public final class VisionConstants {
    public static final Transform3d kExampleCameraToRobotCenter =  new Transform3d(0, 0, 0, new Rotation3d());

    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();  
    public static final int[] kExcludedTags = {};
    public static OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;
    public static boolean updateVision = true; 

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    public static final Vector<N3> kSwerveDrivePoseEstimateTrust = VecBuilder.fill(0.05, 0.05, 0.1);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    public static final Vector<N3> kVisionPoseEstimateTrust = VecBuilder.fill(.4, .4, 0);
    public static final FudgeFactor kFudgeFactor = new FudgeFactor(
    0, 0, 0,
    0, 0, 0);

    public static final double kMaxAmbiguity = .1;
}
