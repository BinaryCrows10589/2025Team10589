package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Subsystems.Climber.ClimberSubsystem.ClimberPosition;

public interface ClimberIO {
    
    @AutoLog
    public static class ClimberIOInputs {
        public double climberPosition = 0.0;
        public double desiredClimberPosition = ClimberSubsystem.resolveClimberPosition(ClimberPosition.RETRACTED);

        public double climberRPM = 0.0;
        public double climberAppliedVolts = 0.0;
        public double[] climberCurrentAmps = new double[] {};
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setDesiredPosition(double desiredPosition) {}

    public default void incrementDesiredPosition(double increment) {}
}
