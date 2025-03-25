package frc.robot.Constants.CameraConstants;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.AutonUtils.AutonPointUtils.FudgeFactor;

// Do not create an instants of a constant class

public final class VisionConstants {
    public static final Transform3d kBackLeftCameraToCenter =  new Transform3d(-Units.inchesToMeters(1.5), -Units.inchesToMeters(13), Units.inchesToMeters(35.5 + .25 + 1.625),  new Rotation3d(0, Units.degreesToRadians(35), Units.degreesToRadians(-135)));//new Transform3d(-Units.inchesToMeters(1.5), -Units.inchesToMeters(13), Units.inchesToMeters(35.5 + .25 + 1.625), new Rotation3d(0, 35, -135));
    public static final Transform3d kBackRightCameraToCenter =  new Transform3d(-Units.inchesToMeters(1.5), Units.inchesToMeters(13.125), Units.inchesToMeters(35.5 + .25 + 1.625), new Rotation3d(0, Units.degreesToRadians(35), Units.degreesToRadians(135)));
    public static final Transform3d kHumanPlayerStationCameraToCenter = new Transform3d(Units.inchesToMeters(5.75), -Units.inchesToMeters(12.1875), Units.inchesToMeters(37.375), new Rotation3d(0, -Units.degreesToRadians(45), Units.degreesToRadians(30)));
//13.25
    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    public static final int[] kExcludedTagsBlue = {};
    public static final int[] kExcludedTagsRed = {};
    public static int[] kExcludedTags = kExcludedTagsBlue;
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
    public static final Vector<N3> kVisionPoseEstimateTrust = VecBuilder.fill(.2, .2, 0);
    public static final FudgeFactor kFudgeFactor = new FudgeFactor(
    0, 0, 0,
    0, 0, 0);

    public static final double kMaxAmbiguity = .02;
    public static final Pose2d kAcceptableVisionDelta = new Pose2d(1, 1, Rotation2d.fromDegrees(10));
}
