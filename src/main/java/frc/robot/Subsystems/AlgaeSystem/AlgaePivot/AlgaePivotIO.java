package frc.robot.Subsystems.AlgaeSystem.AlgaePivot;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotIO {
    
    @AutoLog
    public class AlgaePivotIOInputs {
        public double pivotRPM = 0.0;
        public double rawPivotAngleRotations = 0.0;
        public double offsetPivotAngleRotations = 0.0;
        public double rawDesiredPivotAngleRotations = 0.0;
        public double offsetDesiredPivotAngleRotations = 0.0;
        public double pivotMotorAppliedVolts = 0.0;
        public double[] pivotMotorCurrentAmps = new double[] {};
    }

    public default void updateInputs(AlgaePivotIOInputs inputs) {}

    public default void setDesiredPivotRotation(double desiredRotations) {}

}
