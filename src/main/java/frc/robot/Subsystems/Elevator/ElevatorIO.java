package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.MechanismConstants.ElevatorConstants;

public interface ElevatorIO {
    
    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorRawPosition = 0.0;
        public double elevatorOffsetPosition = 0.0;
        public double rawDesiredElevatorPosition = ElevatorConstants.kDefaultElevatorPosition;
        public double offsetDesiredElevatorPosition = ElevatorConstants.kDefaultElevatorPosition;

        public double elevatorMasterRPM = 0.0;
        public double elevatorMasterAppliedVolts = 0.0;
        public double[] elevatorMasterCurrentAmps = new double[] {};
        public double elevatorMasterMotorTemputureC = 0;

        public double elevatorSlaveRPM = 0.0;
        public double elevatorSlaveAppliedVolts = 0.0;
        public double[] elevatorSlaveCurrentAmps = new double[] {};
        public double elevatorSlaveMotorTemputureC = 0;
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
