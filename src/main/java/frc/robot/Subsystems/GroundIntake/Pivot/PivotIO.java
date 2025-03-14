package frc.robot.Subsystems.GroundIntake.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
        public double pivotRPM = 0.0;
        public double pivotAngleRotationsRaw = 0.0;
        public double pivotAngleRotationsOffset = 0.0;
        public double desiredPivotAngleRotations = 0.0;
        public double offsetDesiredPivotAngleRotations = 0.0;
        public double pivotMotorAppliedVolts = 0.0;
        public double[] pivotMotorCurrentAmps = new double[] {};
    }

    /**
     * Updates all loggable inputes
     * @param inputs PivotIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(PivotIOInputs inputs) {}

    /**
     * Sets the Goal Rotation of the Pivot in Rotations
     * @param desiredRotations Double: The Desired Pivot Rotations
     */
    public default void setDesiredPivotRotation(double desiredRotations) {}

}
