package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    
    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    public ClimberSubsystem(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    public void periodic() {
        this.climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber/", climberInputs);
    }
   
    public void setClimberVoltage(double desiredVoltage) {
        this.climberIO.setDesiredClimberMotorVoltage(desiredVoltage);
    }
 
}
