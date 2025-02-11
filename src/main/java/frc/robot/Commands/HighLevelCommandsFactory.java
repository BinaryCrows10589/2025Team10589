package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AlgaeCommands.AlgaePivotToPositionCommand;
import frc.robot.Commands.AlgaeCommands.IntakeAlgaeCommand;
import frc.robot.Commands.AlgaeCommands.OuttakeAlgaeCommand;
import frc.robot.Commands.AlgaeCommands.IntakeAlgaeCommand;
import frc.robot.Commands.AlgaeCommands.RunAlgaeWheelsCommand;
import frc.robot.Commands.FunnelCommands.DetectFunnelCoralCommand;
import frc.robot.Commands.IntakeCommands.GroundIntakeCommand;
import frc.robot.Commands.IntakeCommands.PivotToPositionCommand;
import frc.robot.Constants.MechanismConstants.AlgaePivotConstants;
import frc.robot.Constants.MechanismConstants.AlgaeWheelConstants;
import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.PivotContants;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotSubsystem;
import frc.robot.Subsystems.AlgaeSystem.AlgaeWheels.AlgaeWheelSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorSubsystem;
import frc.robot.Subsystems.GroundIntake.GroundIntakeCommandFactory;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotSubsystem;
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
    private final PivotSubsystem pivotSubsystem;
    private final AlgaePivotSubsystem algaePivotSubsystem;
    private final AlgaeWheelSubsystem algaeWheelSubsystem;

    private final BooleanSupplier algaeOuttakeButtonSupplier;

    public HighLevelCommandsFactory(
        //GroundIntakeCommandFactory groundIntakeCommandFactory, 
        //TransitWheelsCommandFactory transitWheelsCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory, 
        OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem,
        //TransitCoralSensorSubsystem transitCoralSensorSubsystem,
        FunnelCoralSensorSubsystem funnelCoralSensorSubsystem,
        ElevatorCommandFactory elevatorCommandFactory,
        PivotSubsystem pivotSubsystem,
        AlgaeWheelSubsystem algaeWheelSubsystem,
        AlgaePivotSubsystem algaePivotSubsystem,

        BooleanSupplier algaeOuttakeButtonSupplier
    ) {
        //this.groundIntakeCommandFactory = groundIntakeCommandFactory;
        //this.transitWheelsCommandFactory = transitWheelsCommandFactory;
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
        //this.transitCoralSensorSubsystem = transitCoralSensorSubsystem;
        this.funnelCoralSensorSubsystem = funnelCoralSensorSubsystem;
        this.elevatorCommandFactory = elevatorCommandFactory;
        this.pivotSubsystem = pivotSubsystem;
        this.algaeWheelSubsystem = algaeWheelSubsystem;
        this.algaePivotSubsystem = algaePivotSubsystem;

        this.algaeOuttakeButtonSupplier = algaeOuttakeButtonSupplier;
    }
    
    public GroundIntakeCommand createGroundIntakeCommand() {
        return null;//new GroundIntakeCommand(groundIntakeCommandFactory, transitWheelsCommandFactory, outtakeCommandFactory, outtakeCoralSensorsSubsystem, transitCoralSensorSubsystem, elevatorCommandFactory);
    }
    
    public DetectFunnelCoralCommand createDetectFunnelCoralCommand() {
        return new DetectFunnelCoralCommand(outtakeCommandFactory, outtakeCoralSensorsSubsystem, funnelCoralSensorSubsystem);
    }

    public IntakeAlgaeCommand intakeAlgaeFromReefL2Command() {
        return new IntakeAlgaeCommand(elevatorCommandFactory.createElevatorToReefIntakeAlgaeLowCommand(), new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kReefTreeIntakePositionRotations),
            new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kReefTreeIntakeVoltage),
            this.elevatorCommandFactory, this.algaePivotSubsystem);
    }
    public IntakeAlgaeCommand intakeAlgaeFromReefL3Command() {
        return new IntakeAlgaeCommand(elevatorCommandFactory.createElevatorToReefIntakeAlgaeHighCommand(), new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kReefTreeIntakePositionRotations),
            new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kReefTreeIntakeVoltage),
            this.elevatorCommandFactory, this.algaePivotSubsystem);    
    }
    public IntakeAlgaeCommand intakeAlgaeFromGroundCommand() {
        return new IntakeAlgaeCommand(elevatorCommandFactory.createElevatorToGroundIntakeAlgaeCommand(), new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kReefTreeIntakePositionRotations),
            new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kReefTreeIntakeVoltage),
            this.elevatorCommandFactory, this.algaePivotSubsystem);    }

    public OuttakeAlgaeCommand outtakeAlgaeOnBargeCommand() {
        return new OuttakeAlgaeCommand(elevatorCommandFactory.createElevatorToBargeScoreCommand(),
            new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kOuttakeBargePositionRotations), 
            this.elevatorCommandFactory,
            this.algaePivotSubsystem);
    }

    public OuttakeAlgaeCommand outtakeAlgaeInProcessorCommand() {
        return new OuttakeAlgaeCommand(elevatorCommandFactory.createElevatorToProcessorScoreCommand(),
            new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kOuttakeBargePositionRotations), 
            this.elevatorCommandFactory,
            this.algaePivotSubsystem);
    }

    public RunAlgaeWheelsCommand createAlgaeWheelsIntakeGroundCommand() {
        return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kGroundIntakeVoltage);
    }

    public RunAlgaeWheelsCommand createAlgaeWheelsIntakeReefTreeCommand() {
        return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kReefTreeIntakeVoltage);
    }

    public RunAlgaeWheelsCommand createOutakeWheelsAlgaeBargeCommand() {
        return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kOuttakeBargeVoltage);
    }

    public RunAlgaeWheelsCommand createOutakeWheelsAlgaeProcessorCommand() {
        return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kOuttakeProcessorVoltage);
    }

}

