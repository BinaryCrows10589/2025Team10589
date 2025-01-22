package frc.robot.Subsystems.GroundIntake.IntakeWheels;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWheelsIO {

    @AutoLog
    public static class IntakeWheelsIOInputs {
        public double intakeWheelsRPM = 0.0;
        public double intakeWheelsMotorAppliedVolts = 0.0;
        public double[] intakeWheelsMotorCurrentAmps = new double[] {};
    }

    /**
     * Updates all loggable inputes
     * @param inputs intakeWheelsIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(IntakeWheelsIOInputs inputs) {}

    /**
     * Sets the voltage sent to the IntakeWheels motor
     * @param desiredVoltage Double: The desired voltage of the motor
     */
    public default void setDesiredIntakeWheelsMotorVoltage(double desiredVoltage) {}

}
