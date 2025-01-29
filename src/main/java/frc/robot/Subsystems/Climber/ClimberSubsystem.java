package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Climber.ClimberIOInputsAutoLogged;

public class ClimberSubsystem extends SubsystemBase {
    private ClimberIO climberIO;
    private ClimberIOInputsAutoLogged climberIOInputs;

    public enum ClimberPosition {
        RETRACTED,
        EXTENDED,
        LIFTING
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
        climberIO.setDesiredPosition(desiredPosition);
    }
    public void setDesiredClimberPosition(ClimberPosition desiredPosition) {
        climberIO.setDesiredPosition(resolveClimberPosition(desiredPosition));
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
