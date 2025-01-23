package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitCoralSensor.TransitCoralSensorSubsystem;
import frc.robot.Utils.CommandUtils.Wait;

public class RunIntakeWheelsCommand extends Command {

    private final IntakeWheelsSubsystem intakeWheelsSubsystem;
    private final TransitCoralSensorSubsystem transitCoralSensorSubsystem;
    private final Wait hardCutOffTimer;

    public RunIntakeWheelsCommand(double waitTime, IntakeWheelsSubsystem intakeWheelsSubsystem, TransitCoralSensorSubsystem transitCoralSensorSubsystem) {
        this.intakeWheelsSubsystem = intakeWheelsSubsystem;
        this.transitCoralSensorSubsystem = transitCoralSensorSubsystem;
        this.hardCutOffTimer = new Wait(waitTime);
    }

    @Override
    public void initialize() {
        
        this.hardCutOffTimer.startTimer();
    }
    
}
