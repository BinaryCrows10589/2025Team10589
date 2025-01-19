package frc.robot.Subsystems.Outtake.OuttakeWheels;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeWheelsIO {

    @AutoLog
    public static class OuttakeWheelsIOInputs {
        public double leftWheelRPM;
        public double leftWheelDesired;
        public double rightWheelRPM;
        public double rightWheelDesired;

        public double leftWheelAppliedVolts;
        public double rightWheelAppliedVolts;
        public double[] leftWheelCurrentAmps = new double[] {};
        public double[] rightWheelCurrentAmps = new double[] {};
    }

    public default void updateInputs(OuttakeWheelsIOInputs inputs) {}

    public default void setWheels(double leftWheel, double rightWheel) {}
    
}
