package frc.robot.CrowMotion.UserSide;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.CrowMotion.UserSide.RobotProfilingUtils.CMRobotProfile;

/**
 * Global configuration for CrowMotion.
 * Stores robot physical characteristics and live-state suppliers for use in trajectory planning.
 */
public class CMConfig {
    private static CMRobotProfile robotProfile;

    private static Supplier<double[]> getRobotPositionMetersAndDegrees;
    private static Supplier<double[]> getRobotVelocityMPSandDPS;
    private static Supplier<Double> getAverageSwerveModuleVelocityMPS;
    private static Consumer<double[]> setRobotVelocityMPSANDDPS;
    private static double drivebaseCircumference;

    private static boolean isBlueAlliance;
    private static Supplier<Boolean> defualtShouldMirror;
    private static double fieldWidthMeters;
    private static double fieldLengthMeters;
    private static double defualtMaxDesiredTranslationalVelocity;
    private static double defaultTranslationalAcceleration;
    private static double defaultTranslationalDeceleration;

    private static double defualtMaxDesiredRotationalVelocity;
    private static double defualtMaxDesiredRotationalAcceleration;
    private static double defualtMaxDesiredRotationalDeceleration;
    private static double defaultAngleCorrectionRange;
    private static double defaultMaxRotationCorrectionVelocityDegrees;
    private static double defaultMinRotationVelocityToMove;

    private static double defualtEndTranslationalVelocityForStoppingTrajectories;

    private static double defaultMaxTolorenceDegrees;
    private static double defaultDecelerationBufferDegrees;


    /**
     * Initializes the global CrowMotion configuration.
     */
    public static void init(
        CMRobotProfile _robotProfile,
        Supplier<double[]> _getRobotPositionMetersAndDegrees,
        Supplier<double[]> _getRobotVelocityMPSAndDPS,
        Consumer<double[]> _setRobotVelocityMPSANDDPS,
        double _distanceBetweenCentersOfRightAndLeftWheels,
        double _distanceBetweenCentersOfFrontAndBackWheels,
        boolean _isBlueAlliance,
        Supplier<Boolean> _defaultShouldMirror,
        double _fieldWidthMeters,
        double _fieldLengthMeters,
        double _defualtMaxDesiredTranslationalVelocity,
        double _defaultTranslationalAcceleration,
        double _defaultTranslationalDeceleration,

        double _defualtEndTranslationalVelocityForStoppingTrajectories,
        double _defualtMaxDesiredRotationalVelocity,
        double _defualtMaxDesiredRotationalAcceleration,
        double _defualtMaxDesiredRotationalDeceleration,
        double _defaultAngleCorrectionRange,
        double _defaultMaxRotationCorrectionVelocityDegrees,
        double _defaultMinRotationVelocityToMove,
        double _defaultMaxTolorenceDegrees,
        double _defaultDecelerationBufferDegrees
    ) {
        robotProfile = _robotProfile;
        getRobotPositionMetersAndDegrees = _getRobotPositionMetersAndDegrees;
        getRobotVelocityMPSandDPS = _getRobotVelocityMPSAndDPS;
        setRobotVelocityMPSANDDPS = _setRobotVelocityMPSANDDPS;
        drivebaseCircumference = calculateDrivebaseCircumference(_distanceBetweenCentersOfRightAndLeftWheels, _distanceBetweenCentersOfFrontAndBackWheels);
        isBlueAlliance = _isBlueAlliance;
        defualtShouldMirror = _defaultShouldMirror;
        fieldWidthMeters = _fieldWidthMeters;
        fieldLengthMeters = _fieldLengthMeters;
        
        defualtMaxDesiredTranslationalVelocity = _defualtMaxDesiredTranslationalVelocity;
        defaultTranslationalAcceleration = _defaultTranslationalAcceleration;
        defaultTranslationalDeceleration = _defaultTranslationalDeceleration;
        defualtEndTranslationalVelocityForStoppingTrajectories = _defualtEndTranslationalVelocityForStoppingTrajectories;

        defualtMaxDesiredRotationalVelocity = _defualtMaxDesiredRotationalVelocity;
        defualtMaxDesiredRotationalAcceleration = _defualtMaxDesiredRotationalAcceleration;
        defualtMaxDesiredRotationalDeceleration = _defualtMaxDesiredRotationalDeceleration;
        defaultAngleCorrectionRange = _defaultAngleCorrectionRange;
        defaultMaxRotationCorrectionVelocityDegrees = _defaultMaxRotationCorrectionVelocityDegrees;
        defaultMinRotationVelocityToMove = _defaultMinRotationVelocityToMove;
        defaultMaxTolorenceDegrees = _defaultMaxTolorenceDegrees;
        defaultDecelerationBufferDegrees = _defaultDecelerationBufferDegrees;

        // Sets notifier thread priority for path generation
        Notifier.setHALThreadPriority(true, 50);
    }

    private static double calculateDrivebaseCircumference(double distanceBetweenCentersOfRightAndLeftWheels, double distanceBetweenCentersOfFrontAndBackWheels) {
        double majorAxis = Math.max(distanceBetweenCentersOfRightAndLeftWheels, distanceBetweenCentersOfFrontAndBackWheels);
        double minorAxis = Math.min(distanceBetweenCentersOfRightAndLeftWheels, distanceBetweenCentersOfFrontAndBackWheels);
        double h = Math.pow(((majorAxis - minorAxis) / (majorAxis + minorAxis)), 2);
        
        return (Math.PI * (majorAxis + minorAxis)) * (1 + ((3 * h) / (10 + Math.sqrt(4 - (3 * h)))));
    }

    // ---------------- Getters ----------------

    /** @return Robot profile used for the physics simulations */
    public static CMRobotProfile getRobotProfile() {
        return robotProfile;
    }

    /** @return Robot position [x meters, y meters, rotation degrees]. */
    public static double[] getRobotPositionMetersAndDegrees() {
        return getRobotPositionMetersAndDegrees.get();
    }

    /** @return Robot velocity [x m/s, y m/s, rotation deg/s]. */
    public static double[] getRobotVelocityMPSandDPS() {
        return getRobotVelocityMPSandDPS.get();
    }

    /** @return Average Velocitiy of Swerve modules in MPS*/
    public static double getAverageSwerveModuleVelocityMPS() {
        return getAverageSwerveModuleVelocityMPS.get();
    }

    /**
     * Sets the robot's velocity via wrapped consumer.
     * @param velocity Array of [x m/s, y m/s, rotation deg/s]
     */
    public static void setRobotVelocityMPSandDPS(double xVelocityMPS, double yVelocityMPS, double rotationalVelocityDPS) {
        setRobotVelocityMPSANDDPS.accept(new double[] {xVelocityMPS, yVelocityMPS, rotationalVelocityDPS});
    }

    /** @return The wheel circumference in meters */
    public static double getDrivebaseCircumference() {
        return drivebaseCircumference;
    }

    /** @return If the current allience is the blue alliance */
    public static boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    /** @return Whether the path should be mirrored. */
    public static boolean getShouldMirror() {
        return defualtShouldMirror.get();
    }

    /** @return The field width in meters */
    public static double getFieldWidth() {
        return fieldWidthMeters;
    }

    /** @return The field length in meters */
    public static double getFieldLength() {
        return fieldLengthMeters;
    }

    /** @return Maximum translational velocity to target. */
    public static double getDefaultMaxDesiredTranslationalVelocity() {
        return defualtMaxDesiredTranslationalVelocity;
    }

    /** @return Maximum rotational velocity to target. */
    public static double getDefaultMaxDesiredRotationalVelocity() {
        return defualtMaxDesiredRotationalVelocity;
    }

    public static double getDefaultTranslationalAcceleration() {
        return defaultTranslationalAcceleration;
    }
    
    public static double getDefaultTranslationalDeceleration() {
        return defaultTranslationalDeceleration;
    }
    
    /** @return Desired end translational velocity when stopping. */
    public static double getDefaultEndTranslationalVelocityForStoppingTrajectories() {
        return defualtEndTranslationalVelocityForStoppingTrajectories;
    }

    public static double getDefaultMaxDesiredRotationalAcceleration() {
        return defualtMaxDesiredRotationalAcceleration;
    }

    public static double getDefaultMaxDesiredRotationalDeceleration() {
        return defualtMaxDesiredRotationalDeceleration;
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

    public static double getDefaultMaxTolorenceDegrees() {
        return defaultMaxTolorenceDegrees;
    }

    public static double getDefaultDecelerationBufferDegrees() {
        return defaultDecelerationBufferDegrees;
    }
}
