package frc.robot.CrowMotion;

import java.util.function.Consumer;
import java.util.function.Supplier;

import frc.robot.CrowMotion.RobotProfilingUtils.RobotProfile;

/**
 * Global configuration for CrowMotion.
 * Stores robot physical characteristics and live-state suppliers for use in trajectory planning.
 */
public class CrowMotionConfig {


    private static RobotProfile robotProfile;

    private static Supplier<double[]> getRobotPositionMetersAndDegrees;
    private static Supplier<double[]> getRobotVelocityMPSandDPS;
    private static Supplier<Double> getAverageSwerveModuleVelocityMPS;
    private static Consumer<double[]> setRobotVelocityMPSANDDPS;

    private static Supplier<Boolean> defualtShouldMirror;
    private static double defualtMaxDesiredTranslationalVelocity;
    private static double defualtMaxDesiredRotationalVelocity;
    private static double defualtEndTranslationalVelocityForStoppingTrajectories;

    /**
     * Initializes the global CrowMotion configuration.
     *
     * @param robotProfile Robot physical configuration profile. Reference the RobotProfile file for help creating the profile. 
     * @param _getRobotPositionMetersAndDegrees Supplier for [x, y, rot] position
     * @param _getRobotVelocityMPSAndDPS Supplier for [x, y, rot] velocity
     * @param _getAverageSwerveModuleVelocityMPS Supplier for average swerve module veclocity
     * @param _setRobotVelocityMPSANDDPS Consumer for setting [x, y, rot] velocity
     * @param _defaultShouldMirror Supplier to determine if path should be mirrored
     * @param _defualtMaxDesiredTranslationalVelocity Maximum translational velocity to target
     * @param _defualtMaxDesiredRotationalVelocity Maximum rotational velocity to target
     * @param _defualtEndTranslationalVelocityForStoppingTrajectories Desired end velocity for stop paths
     */
    public static void init(
        RobotProfile _robotProfile,
        Supplier<double[]> _getRobotPositionMetersAndDegrees,
        Supplier<double[]> _getRobotVelocityMPSAndDPS,
        Supplier<Double> _getAverageSwerveModuleVelocityMPS,
        Consumer<double[]> _setRobotVelocityMPSANDDPS,
        Supplier<Boolean> _defaultShouldMirror,
        double _defualtMaxDesiredTranslationalVelocity,
        double _defualtMaxDesiredRotationalVelocity,
        double _defualtEndTranslationalVelocityForStoppingTrajectories
    ) {
        robotProfile = _robotProfile;
        getRobotPositionMetersAndDegrees = _getRobotPositionMetersAndDegrees;
        getRobotVelocityMPSandDPS = _getRobotVelocityMPSAndDPS;
        getAverageSwerveModuleVelocityMPS = _getAverageSwerveModuleVelocityMPS;
        setRobotVelocityMPSANDDPS = _setRobotVelocityMPSANDDPS;
        defualtShouldMirror = _defaultShouldMirror;
        defualtMaxDesiredTranslationalVelocity = _defualtMaxDesiredTranslationalVelocity;
        defualtMaxDesiredRotationalVelocity = _defualtMaxDesiredRotationalVelocity;
        defualtEndTranslationalVelocityForStoppingTrajectories = _defualtEndTranslationalVelocityForStoppingTrajectories;
    }

    /** @return Robot profile used for the physics simulations */
    public static RobotProfile getRobotProfile() {
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

    /** @return Whether the path should be mirrored. */
    public static boolean getShouldMirror() {
        return defualtShouldMirror.get();
    }

    /** @return Maximum translational velocity to target. */
    public static double getDefaultMaxDesiredTranslationalVelocity() {
        return defualtMaxDesiredTranslationalVelocity;
    }

    /** @return Maximum rotational velocity to target. */
    public static double getDefaultMaxDesiredRotationalVelocity() {
        return defualtMaxDesiredRotationalVelocity;
    }

    /** @return Desired end translational velocity when stopping. */
    public static double getDefaultEndTranslationalVelocityForStoppingTrajectories() {
        return defualtEndTranslationalVelocityForStoppingTrajectories;
    }
}
