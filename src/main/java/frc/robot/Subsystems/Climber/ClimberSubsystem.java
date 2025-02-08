package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Climber.ClimberIOInputsAutoLogged;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem.ElevatorMode;
import frc.robot.Utils.GeneralUtils.Tolerance;

public class ClimberSubsystem extends SubsystemBase {
    private ClimberIO climberIO;
    private ClimberIOInputsAutoLogged climberIOInputs;

    ClimberControlMode currentMode = ClimberControlMode.AUTOMATIC_POSITIONING;

    public enum ClimberPosition {
        RETRACTED,
        EXTENDED,
        LIFTING
    }

    public enum ClimberControlMode {
        AUTOMATIC_POSITIONING,
        MANUAL
    }

    public ClimberSubsystem(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    @Override
    public void periodic() {
        this.climberIO.updateInputs(this.climberIOInputs);
        Logger.processInputs("Climber/", climberIOInputs);
    }

    public void setDesiredClimberPosition(double desiredPosition) {
        updateClimberControlMode(ClimberControlMode.AUTOMATIC_POSITIONING);
        climberIO.setDesiredPosition(desiredPosition);
    }
    public void setDesiredClimberPosition(ClimberPosition desiredPosition) {
        updateClimberControlMode(ClimberControlMode.AUTOMATIC_POSITIONING);
        climberIO.setDesiredPosition(resolveClimberPosition(desiredPosition));
    }

    public void incrementDesiredClimberPosition(double increment) {
        updateClimberControlMode(ClimberControlMode.MANUAL);
        climberIO.incrementDesiredPosition(increment);
    }

    public boolean isClimberInTolerance(double tolerance) {
        return Tolerance.inTolorance(this.climberIOInputs.climberPosition, this.climberIOInputs.desiredClimberPosition,
        tolerance);
    }

    public void updateClimberControlMode(ClimberControlMode controlMode) {
        this.currentMode = controlMode;
    }

    public static double resolveClimberPosition(ClimberPosition desiredPosition) {
        switch (desiredPosition) {
            case RETRACTED:
                return 0;
            case EXTENDED:
                return 0;
            case LIFTING:
                return 0;
            default:
                return 0;
        }
    }
}
