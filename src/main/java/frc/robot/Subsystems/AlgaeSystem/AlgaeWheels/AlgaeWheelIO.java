package frc.robot.Subsystems.AlgaeSystem.AlgaeWheels;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeWheelIO {
    @AutoLog
    public static class AlgaeWheelIOInputs {
        public double wheelRPM;
        public double wheelDesiredVoltage;
        public double wheelDesiredPosition;

        public double wheelAppliedVolts;
        public double[] wheelCurrentAmps = new double[] {};
    }

    public default void updateInputs(AlgaeWheelIOInputs inputs) {}

    public default void setWheelPosition(double desiredPosition) {}
    public default void setWheelPositionRelative(double desiredPosition) {}
    public default void setWheelVoltage(double voltage) {}
    
}
