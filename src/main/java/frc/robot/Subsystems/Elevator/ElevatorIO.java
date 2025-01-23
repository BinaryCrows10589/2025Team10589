package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.MechanismConstants.ElevatorConstants;

public interface ElevatorIO {
    
    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorPosition = 0.0;
        public double desiredElevatorPosition = ElevatorConstants.kDefaultElevatorPosition;

        public double elevatorRPM = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps = new double[] {};
    }

    /**
     * Updates all loggable inputes
     * @param inputs ElevatorIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /**
     * Sets the goal position of the elevator //TODO: Units
     * @param desiredPosition Double: The desired position of the elevator
     */
    public default void setDesiredPosition(double desiredPosition) {}

    public default void incrementDesiredPosition(double increment) {}
    
}
