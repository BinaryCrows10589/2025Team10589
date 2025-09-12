package frc.robot.CrowMotion.UserSide;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * CMConfig is a centralized configuration class for CrowMotion.
 * <p>
 * This class stores robot, field, and trajectory parameters, and provides static accessors
 * and mutators for consumers and suppliers of these values. All values must be initialized
 * through the {@link #init(Supplier, Supplier, Consumer, double, boolean, Supplier, double, double, 
 * double, double, double, double, double, double, double, double, double, double, double, double, double, double)} method
 * before usage.
 * </p>
 */
public class CMConfig {

    // ------------------------------
    // Robot state suppliers & consumers
    // ------------------------------
    private static Supplier<double[]> getRobotPositionMetersAndDegrees;
    private static Supplier<double[]> getRobotVelocityMPSAndDPS;
    private static Consumer<double[]> setRobotVelocityMPSAndDPS;

    // ------------------------------
    // Physical parameters
    // ------------------------------
    private static double drivebaseCircumference;

    // ------------------------------
    // Field and alliance info
    // ------------------------------
    private static boolean isBlueAlliance;
    private static Supplier<Boolean> defaultShouldMirror;
    private static double fieldWidthMeters;
    private static double fieldLengthMeters;

    // ------------------------------
    // Translational motion defaults
    // ------------------------------
    private static double defaultMaxDesiredTranslationalVelocity;
    private static double defaultTranslationalAcceleration;
    private static double defaultTranslationalDeceleration;

    // ------------------------------
    // Rotational motion defaults
    // ------------------------------
    private static double defaultMaxDesiredRotationalVelocity;
    private static double defaultMaxDesiredRotationalAcceleration;
    private static double defaultMaxDesiredRotationalDeceleration;
    private static double defaultAngleCorrectionRange;
    private static double defaultMaxRotationCorrectionVelocityDegrees;
    private static double defaultMinRotationVelocityToMove;

    // ------------------------------
    // Trajectory stopping behavior
    // ------------------------------
    private static double defaultEndTranslationalVelocityForStoppingTrajectories;

    // ------------------------------
    // Tolerance & deceleration
    // ------------------------------
    private static double defaultMaxToleranceDegrees;
    private static double defaultDecelerationBufferDegrees;
    private static double defaultRotationSettleTime;

    // ------------------------------
    // Reason / extra variables
    // ------------------------------
    private static double maxPossibleAverageSwerveModuleMPS;

    /**
     * <p>
     * Initializes Crow Motion and sets default values for trejectories.
     * </p>
     *
     * @param robotPositionSupplier Supplier providing robot position in meters and degrees [x, y, angle]
     * @param robotVelocitySupplier Supplier providing robot velocity in meters/sec and degrees/sec [vx, vy, omega]
     * @param robotVelocityConsumer Consumer to set robot velocity in meters/sec and degrees/sec [vx, vy, omega]
     * @param distanceBetweenCentersOfRightAndLeftWheels Distence between centers of right and left wheels
     * @param distanceBetweenCentersOfFrontAndBackWheels Distance between centers of front and back wheels
     * @param isBlueAlliance Boolean indicating if the robot is on the blue alliance
     * @param shouldMirrorSupplier Supplier to determine if paths should be mirrored
     * @param fieldWidth Field width in meters
     * @param fieldLength Field length in meters
     * @param maxTranslationalVelocity Maximum desired translational velocity (m/s)
     * @param translationalAcceleration Translational acceleration (m/s²)
     * @param translationalDeceleration Translational deceleration (m/s²)
     * @param maxRotationalVelocity Maximum desired rotational velocity (deg/s)
     * @param maxRotationalAcceleration Maximum desired rotational acceleration (deg/s²)
     * @param maxRotationalDeceleration Maximum desired rotational deceleration (deg/s²)
     * @param angleCorrectionRange Angle correction range (degrees)
     * @param maxRotationCorrectionVelocity Maximum rotational correction velocity (deg/s)
     * @param minRotationVelocityToMove Minimum rotational velocity to start moving (deg/s)
     * @param endTranslationalVelocityForStopping Trajectory stopping velocity (m/s)
     * @param maxToleranceDegrees Maximum allowed angle tolerance for trajectory (deg)
     * @param decelerationBufferDegrees Buffer for deceleration calculations (deg)
     * @param rotationSettleTime Rotation settle time (seconds)
     * @param maxSwerveModuleMPS Maximum possible average swerve module velocity (m/s)
     */
    public static void init(
            Supplier<double[]> robotPositionSupplier,
            Supplier<double[]> robotVelocitySupplier,
            Consumer<double[]> robotVelocityConsumer,
            double distanceBetweenCentersOfRightAndLeftWheels,
            double distanceBetweenCentersOfFrontAndBackWheels,
            boolean isBlueAlliance,
            Supplier<Boolean> shouldMirrorSupplier,
            double fieldWidth,
            double fieldLength,
            double maxTranslationalVelocity,
            double translationalAcceleration,
            double translationalDeceleration,
            double maxRotationalVelocity,
            double maxRotationalAcceleration,
            double maxRotationalDeceleration,
            double angleCorrectionRange,
            double maxRotationCorrectionVelocity,
            double minRotationVelocityToMove,
            double endTranslationalVelocityForStopping,
            double maxToleranceDegrees,
            double decelerationBufferDegrees,
            double rotationSettleTime,
            double maxSwerveModuleMPS
    ) {
        CMConfig.getRobotPositionMetersAndDegrees = robotPositionSupplier;
        CMConfig.getRobotVelocityMPSAndDPS = robotVelocitySupplier;
        CMConfig.setRobotVelocityMPSAndDPS = robotVelocityConsumer;

        CMConfig.drivebaseCircumference = CMConfig.calculateDrivebaseCircumference(distanceBetweenCentersOfRightAndLeftWheels, distanceBetweenCentersOfFrontAndBackWheels);

        CMConfig.isBlueAlliance = isBlueAlliance;
        CMConfig.defaultShouldMirror = shouldMirrorSupplier;
        CMConfig.fieldWidthMeters = fieldWidth;
        CMConfig.fieldLengthMeters = fieldLength;

        CMConfig.defaultMaxDesiredTranslationalVelocity = maxTranslationalVelocity;
        CMConfig.defaultTranslationalAcceleration = translationalAcceleration;
        CMConfig.defaultTranslationalDeceleration = translationalDeceleration;

        CMConfig.defaultMaxDesiredRotationalVelocity = maxRotationalVelocity;
        CMConfig.defaultMaxDesiredRotationalAcceleration = maxRotationalAcceleration;
        CMConfig.defaultMaxDesiredRotationalDeceleration = maxRotationalDeceleration;
        CMConfig.defaultAngleCorrectionRange = angleCorrectionRange;
        CMConfig.defaultMaxRotationCorrectionVelocityDegrees = maxRotationCorrectionVelocity;
        CMConfig.defaultMinRotationVelocityToMove = minRotationVelocityToMove;

        CMConfig.defaultEndTranslationalVelocityForStoppingTrajectories = endTranslationalVelocityForStopping;

        CMConfig.defaultMaxToleranceDegrees = maxToleranceDegrees;
        CMConfig.defaultDecelerationBufferDegrees = decelerationBufferDegrees;
        CMConfig.defaultRotationSettleTime = rotationSettleTime;

        CMConfig.maxPossibleAverageSwerveModuleMPS = maxSwerveModuleMPS;
    }

    private static double calculateDrivebaseCircumference(double distanceBetweenCentersOfRightAndLeftWheels, double distanceBetweenCentersOfFrontAndBackWheels) {
        double majorAxis = Math.max(distanceBetweenCentersOfRightAndLeftWheels, distanceBetweenCentersOfFrontAndBackWheels);
        double minorAxis = Math.min(distanceBetweenCentersOfRightAndLeftWheels, distanceBetweenCentersOfFrontAndBackWheels);
        double h = Math.pow(((majorAxis - minorAxis) / (majorAxis + minorAxis)), 2);
        
        return (Math.PI * (majorAxis + minorAxis)) * (1 + ((3 * h) / (10 + Math.sqrt(4 - (3 * h)))));
    }
    
    public static double[] getRobotPositionMetersAndDegrees() {
        return getRobotPositionMetersAndDegrees.get();
    }

    public static double[] getRobotVelocityMPSAndDPS() {
        return getRobotVelocityMPSAndDPS.get();
    }

    public static void setRobotVelocityMPSAndDPS(double x, double y, double rot) {
        setRobotVelocityMPSAndDPS.accept(new double[] {x, y, rot});
    }

    public static double getDrivebaseCircumference() {
        return drivebaseCircumference;
    }

    public static boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    public static boolean shouldMirror() {
        return defaultShouldMirror.get();
    }

    public static double getFieldWidthMeters() {
        return fieldWidthMeters;
    }

    public static double getFieldLengthMeters() {
        return fieldLengthMeters;
    }

    public static double getDefaultMaxDesiredTranslationalVelocity() {
        return defaultMaxDesiredTranslationalVelocity;
    }

    public static double getDefaultTranslationalAcceleration() {
        return defaultTranslationalAcceleration;
    }

    public static double getDefaultTranslationalDeceleration() {
        return defaultTranslationalDeceleration;
    }

    public static double getMaxPossibleAverageSwerveModuleMPS() {
        return maxPossibleAverageSwerveModuleMPS;
    }

    public static double getDefaultMaxDesiredRotationalVelocity() {
        return defaultMaxDesiredRotationalVelocity;
    }

    public static double getDefaultMaxDesiredRotationalAcceleration() {
        return defaultMaxDesiredRotationalAcceleration;
    }

    public static double getDefaultMaxDesiredRotationalDeceleration() {
        return defaultMaxDesiredRotationalDeceleration;
    }

    public static double getDefaultAngleCorrectionRange() {
        return defaultAngleCorrectionRange;
    }

    public static double getDefaultMaxRotationCorrectionVelocityDegrees() {
        return defaultMaxRotationCorrectionVelocityDegrees;
    }

    public static double getDefaultMinRotationVelocityToMove() {
        return defaultMinRotationVelocityToMove;
    }

    public static double getDefaultEndTranslationalVelocityForStoppingTrajectories() {
        return defaultEndTranslationalVelocityForStoppingTrajectories;
    }

    public static double getDefaultMaxToleranceDegrees() {
        return defaultMaxToleranceDegrees;
    }

    public static double getDefaultDecelerationBufferDegrees() {
        return defaultDecelerationBufferDegrees;
    }

    public static double getDefaultRotationSettleTime() {
        return defaultRotationSettleTime;
    }
}
