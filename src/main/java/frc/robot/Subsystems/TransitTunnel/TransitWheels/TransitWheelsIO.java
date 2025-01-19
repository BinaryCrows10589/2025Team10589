package frc.robot.Subsystems.TransitTunnel.TransitWheels;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIO.SwerveModuleIOInputs;

public interface TransitWheelsIO {

    @AutoLog
    public static class TransitWheelsIOInputs {
        public double transitWheelsRPM = 0.0;
        public double transitWheelsMotorAppliedVolts = 0.0;
        public double[] transitWheelsMotorCurrentAmps = new double[] {};
    }

    /**
     * Updates all loggable inputes
     * @param inputs TransitWheelsIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(TransitWheelsIOInputs inputs) {}

    /**
     * Sets the voltage sent to the TransitWheels motor
     * @param desiredVoltage Double: The desired voltage of the motor
     */
    public default void setDesiredTransitWheelsMotorVoltage(double desiredVoltage) {}

}
