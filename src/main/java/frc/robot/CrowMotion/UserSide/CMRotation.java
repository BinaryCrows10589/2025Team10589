package frc.robot.CrowMotion.UserSide;

public class CMRotation {
    private double angleDegrees;
    private int rotationDirrection;
    private double completeRotationPercent;
    private double maxRotationVelocityDegrees;
    private double desiredRotationalAccelerationDegrees;
    private double desiredRotationDecelerationDegrees;
    private double angleCorrectionRange;
    private double maxRotationCorrectionVelocityDegrees;
    private double minRotationVelocityToMoveDegrees;
    private double maxTolorenceDegrees;
    private double decelerationBufferDegrees;
    private boolean shouldMirror = false;
    

    /**
     * Constructs a CMRotation with optional field mirroring applied.
     *
     * @param angleDegrees  The angle of rotation in degrees
     * @param rotationDirrection The direction of the rotation, -1 or 1 or 0(shortest)
     * @param completeRotationPercent The percent of the path by which rotation should be completed (0.0 to 1.0)
     * @param shouldMirror Wether the direction and angle should be mirrored (e.g., for alliance side switching)
     */
    public CMRotation(double angleDegrees, int rotationDirrection, double completeRotationPercent, 
        double maxRotationVelocityDegrees, double desiredRotationalAccelerationDegrees, double desiredRotationDecelerationDegrees, 
        double angleCorrectionRange, double maxRotationCorrectionVelocityDegrees, double minRotationVelocityToMoveDegrees,
        double maxTolorenceDegrees, double decelerationBufferDegrees,
        boolean shouldMirror) {
            
        this.angleDegrees = angleDegrees;
        assert (rotationDirrection == -1 || rotationDirrection == 1 || rotationDirrection == 0) : "Crow Motion, for rotation with angle of " + angleDegrees + " degrees. Rotation Direction must be -1 or 1 or 0(shortest): " + rotationDirrection + " is invalid";   
        this.rotationDirrection = rotationDirrection;
        this.completeRotationPercent = completeRotationPercent;
        this.maxRotationVelocityDegrees = maxRotationVelocityDegrees;
        this.desiredRotationalAccelerationDegrees = maxRotationVelocityDegrees;
        this.desiredRotationDecelerationDegrees = desiredRotationDecelerationDegrees;
        this.angleCorrectionRange = angleCorrectionRange;
        this.maxRotationCorrectionVelocityDegrees = maxRotationCorrectionVelocityDegrees;
        this.minRotationVelocityToMoveDegrees = minRotationVelocityToMoveDegrees;
        this.maxTolorenceDegrees = maxTolorenceDegrees;
        this.decelerationBufferDegrees = decelerationBufferDegrees;
        this.shouldMirror = shouldMirror;
    }

    /**
     * Constructs a CMRotation with the default field mirror set through CrowMotionConfig
     *
     * @param angleDegrees The angle of rotation in degrees
     * @param rotationDirrection The direction of the rotation, -1 or 1 or 0(shortest)
     * @param completeRotationPercent The percent of the path by which rotation should be completed (0.0 to 1.0)
     */
    public CMRotation(double angleDegrees, int rotationDirrection,
        double completeRotationPercent, double maxRotationVelocityDegrees,
        double desiredRotationalAccelerationDegrees,
        double desiredRotationDecelerationDegrees, 
        double angleCorrectionRange, double maxRotationCorrectionVelocityDegrees,
        double minRotationVelocityToMoveDegrees,
        double maxTolorenceDegrees, double decelerationBufferDegrees) {
        this(angleDegrees, rotationDirrection,
            completeRotationPercent,
            maxRotationVelocityDegrees, 
            desiredRotationalAccelerationDegrees,
            desiredRotationDecelerationDegrees,
            angleCorrectionRange, maxRotationCorrectionVelocityDegrees,
            minRotationVelocityToMoveDegrees,
            maxTolorenceDegrees, decelerationBufferDegrees,
            CMConfig.getShouldMirror());
    }
    

    /**
     * Gets the angle of rotation in degrees.
     *
     * @return the angle in degrees
     */
    public double getAngleDegrees() {
        return wrapAngle(shouldMirror ? this.angleDegrees * -1 : this.angleDegrees);
    }

    /**
     * Gets the direction of rotation.
     *
     * @return the rotation direction as a RotationDirrection enum
     */
    public int getRotationDirrection() {
        return shouldMirror ? (rotationDirrection == 1 ? -1 : 1)
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

    public double getMaxRotationVelocityDegrees() {
        return maxRotationVelocityDegrees;
    }
    
    public double getDesiredRotationalAccelerationDegrees() {
        return desiredRotationalAccelerationDegrees;
    }
    
    public double getDesiredRotationalDecelerationDegrees() {
        return desiredRotationDecelerationDegrees;
    }

    public double getAngleCorrectionRange() {
        return this.angleCorrectionRange;
    }

    public static double wrapAngle(double angleDegrees) {
        return (angleDegrees + 180) % 360 - 180;
    }

    public double getMaxRotationCorrectionVelocityDegrees() {
        return this.maxRotationCorrectionVelocityDegrees;
    }

    public double getMinRotationVelocityToMoveDegrees() {
        return this.minRotationVelocityToMoveDegrees;
    }

    public double getMaxTolorenceDegrees() {
        return this.maxTolorenceDegrees;
    }

    public double getDecelerationBufferDegrees() {
        return this.decelerationBufferDegrees;
    }
}
