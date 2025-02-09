package frc.robot.Subsystems.Outtake.OuttakeWheels;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeWheelsIO {

    @AutoLog
    public static class OuttakeWheelsIOInputs {
        public double leftWheelRPM;
        public double leftWheelDesiredVoltage;
        public double leftWheelDesiredPosition;
        public double rightWheelRPM;
        public double rightWheelDesiredVoltage;
        public double rightWheelDesiredPosition;

        public double leftWheelAppliedVolts;
        public double rightWheelAppliedVolts;
        public double[] leftWheelCurrentAmps = new double[] {};
        public double[] rightWheelCurrentAmps = new double[] {};
    }

    public default void updateInputs(OuttakeWheelsIOInputs inputs) {}

    public default void setWheelPositions(double rightWheel) {}
    public default void setWheelPositionsRelative(double rightWheel) {}
    public default void setWheelVoltages( double rightWheel) {}
    
}
