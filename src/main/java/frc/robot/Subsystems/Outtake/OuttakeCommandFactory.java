package frc.robot.Subsystems.Outtake;

import frc.robot.Commands.OuttakeWheelsCommands.HoldCoralInOuttakeCommand;
import frc.robot.Commands.OuttakeWheelsCommands.OuttakeCoralCommand;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsSubsystem;

public class OuttakeCommandFactory {
    
    private final OuttakeWheelsSubsystem outtakeWheelsSubsystem;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;

    public OuttakeCommandFactory(OuttakeWheelsSubsystem outtakeWheelsSubsystem, OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        this.outtakeWheelsSubsystem = outtakeWheelsSubsystem;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
    }
    
    public HoldCoralInOuttakeCommand createHoldCoralInOuttakeCommand() {
        return new HoldCoralInOuttakeCommand(outtakeWheelsSubsystem, outtakeCoralSensorsSubsystem);
    }

    public OuttakeCoralCommand createOuttakeCoralCommand() {
        return new OuttakeCoralCommand(outtakeWheelsSubsystem, outtakeCoralSensorsSubsystem);
    }
}
