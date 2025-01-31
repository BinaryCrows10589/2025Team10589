package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.FunnelCommands.DetectFunnelCoralCommand;
import frc.robot.Commands.IntakeCommands.GroundIntakeCommand;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorSubsystem;
import frc.robot.Subsystems.GroundIntake.GroundIntakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitCoralSensor.TransitCoralSensorSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsCommandFactory;

public class HighLevelCommandsFactory {

    //private final GroundIntakeCommandFactory groundIntakeCommandFactory;
    //private final TransitWheelsCommandFactory transitWheelsCommandFactory;
    private final OuttakeCommandFactory outtakeCommandFactory;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;
    //private final TransitCoralSensorSubsystem transitCoralSensorSubsystem;
    private final FunnelCoralSensorSubsystem funnelCoralSensorSubsystem;
    private final ElevatorCommandFactory elevatorCommandFactory;

    public HighLevelCommandsFactory(
        //GroundIntakeCommandFactory groundIntakeCommandFactory, 
        //TransitWheelsCommandFactory transitWheelsCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory, 
        OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem,
        //TransitCoralSensorSubsystem transitCoralSensorSubsystem,
        FunnelCoralSensorSubsystem funnelCoralSensorSubsystem,
        ElevatorCommandFactory elevatorCommandFactory
    ) {
        //this.groundIntakeCommandFactory = groundIntakeCommandFactory;
        //this.transitWheelsCommandFactory = transitWheelsCommandFactory;
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
        //this.transitCoralSensorSubsystem = transitCoralSensorSubsystem;
        this.funnelCoralSensorSubsystem = funnelCoralSensorSubsystem;
        this.elevatorCommandFactory = elevatorCommandFactory;
    }
    
    public GroundIntakeCommand createGroundIntakeCommand() {
        return null;//new GroundIntakeCommand(groundIntakeCommandFactory, transitWheelsCommandFactory, outtakeCommandFactory, outtakeCoralSensorsSubsystem, transitCoralSensorSubsystem, elevatorCommandFactory);
    }
    
    public DetectFunnelCoralCommand createDetectFunnelCoralCommand() {
        return new DetectFunnelCoralCommand(outtakeCommandFactory, outtakeCoralSensorsSubsystem, funnelCoralSensorSubsystem);
    }
}
