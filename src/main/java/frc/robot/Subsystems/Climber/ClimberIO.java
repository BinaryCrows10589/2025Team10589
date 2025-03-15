package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
        public double climberRPM = 0.0;
        public double climberMotorAppliedVolts = 0.0;
        public double climberDesiredVoltage = 0.0;
        public double[] climberMotorCurrentAmps = new double[] {};
    }

    /**
     * Updates all loggable inputes
     * @param inputs ClimberIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(ClimberIOInputs inputs) {}

    /**
     * Sets the voltage sent to the climber motor
     * @param desiredVoltage Double: The desired voltage of the motor
     */
    public default void setDesiredClimberMotorVoltage(double desiredVoltage) {}

}
