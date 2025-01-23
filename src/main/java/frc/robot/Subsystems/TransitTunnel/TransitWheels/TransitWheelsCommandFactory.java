package frc.robot.Subsystems.TransitTunnel.TransitWheels;

import frc.robot.Commands.TransitCommands.RunTransitToOuttakeCommand;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;

public class TransitWheelsCommandFactory {
    
    private final TransitWheelsSubsystem transitWheelsSubsystem;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;

    public TransitWheelsCommandFactory(TransitWheelsSubsystem transitWheelsSubsystem, OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        this.transitWheelsSubsystem = transitWheelsSubsystem;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
    }

    public RunTransitToOuttakeCommand createRunTransitToOuttakeCommand() {
        return new RunTransitToOuttakeCommand(transitWheelsSubsystem, outtakeCoralSensorsSubsystem);
    }

    
}
