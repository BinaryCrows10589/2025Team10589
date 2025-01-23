package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.IntakeWheelsConstants;
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
        addRequirements(intakeWheelsSubsystem, transitCoralSensorSubsystem);
    }


    @Override
    public void initialize() {
        intakeWheelsSubsystem.setDesiredIntakeWheelsVoltage(IntakeWheelsConstants.kIntakeWheelsIntakeVoltage);
        this.hardCutOffTimer.startTimer();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intakeWheelsSubsystem.setDesiredIntakeWheelsVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return this.transitCoralSensorSubsystem.isCoralInTransit() || hardCutOffTimer.hasTimePassed();
    }
    
}
