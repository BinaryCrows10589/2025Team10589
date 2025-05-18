package frc.robot.CrowMotion;

public class CMRotation {
    
    public static enum RotationDirrection {
        POSITIVE,
        NEGITIVE
    }

    private double angleDegrees;
    private RotationDirrection rotationDirrection;
    private double completeRotationPercent;
    private boolean shouldMirror = false;
    //TODO: Fix rotation as mirroing must happen in runtime not in compile time
    /**
     * Constructs a CMRotation with optional field mirroring applied.
     *
     * @param angleDegrees  The angle of rotation in degrees
     * @param rotationDirrection The direction of the rotation (POSITIVE or NEGITIVE)
     * @param completeRotationPercent The percent of the path by which rotation should be completed (0.0 to 1.0)
     * @param shouldMirror Wether the direction and angle should be mirrored (e.g., for alliance side switching)
     */
    public CMRotation(double angleDegrees, RotationDirrection rotationDirrection, double completeRotationPercent, boolean shouldMirror) {
        this.angleDegrees = (angleDegrees + 180) % 360 - 180;
        this.rotationDirrection = rotationDirrection;
        this.completeRotationPercent = completeRotationPercent;
        this.shouldMirror = shouldMirror;
    }

    /**
     * Constructs a CMRotation with the default field mirror set through CrowMotionConfig
     *
     * @param angleDegrees The angle of rotation in degrees
     * @param rotationDirrection The direction of the rotation (POSITIVE or NEGITIVE)
     * @param completeRotationPercent The percent of the path by which rotation should be completed (0.0 to 1.0)
     */
    public CMRotation(double angleDegrees, RotationDirrection rotationDirrection, double completeRotationPercent) {
        this(angleDegrees, rotationDirrection, completeRotationPercent, CrowMotionConfig.getShouldMirror());
    }

    /**
     * Gets the angle of rotation in degrees.
     *
     * @return the angle in degrees
     */
    public double angleDegrees() {
        return shouldMirror ? this.angleDegrees * -1 : this.angleDegrees;
    }

    /**
     * Gets the direction of rotation.
     *
     * @return the rotation direction as a RotationDirrection enum
     */
    public RotationDirrection getRotationDirrection() {
        return shouldMirror ? (rotationDirrection == RotationDirrection.POSITIVE ? RotationDirrection.NEGITIVE : RotationDirrection.POSITIVE)
        : rotationDirrection;
    }

    /**
     * Gets the percent of the path by which rotation should be completed.
     *
     * @return the percent (0.0 to 1.0)
     */
    public double getCompleteRotationPercent() {
        return this.completeRotationPercent;
    }

}
