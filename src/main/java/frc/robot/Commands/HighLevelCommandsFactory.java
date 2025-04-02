package frc.robot.Commands;

import frc.robot.Commands.AlgaeCommands.AlgaePivotToPositionCommand;
import frc.robot.Commands.AlgaeCommands.IntakeAlgaeCommand;
import frc.robot.Commands.AlgaeCommands.OuttakeAlgaeCommand;
import frc.robot.Commands.AlgaeCommands.RunAlgaeWheelsCommand;
import frc.robot.Commands.AutoPositionCommands.ScrollThanOuttakeCommand;
import frc.robot.Commands.AutoPositionCommands.ScrollWithReefTreeDetectorCommand;
import frc.robot.Commands.FunnelCommands.DetectFunnelCoralCommand;
import frc.robot.Commands.IntakeCommands.GroundIntakeCommand;
import frc.robot.Constants.GenericConstants.AutoPositionConstants;
import frc.robot.Constants.MechanismConstants.AlgaePivotConstants;
import frc.robot.Constants.MechanismConstants.AlgaeWheelConstants;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotSubsystem;
import frc.robot.Subsystems.AlgaeSystem.AlgaeWheels.AlgaeWheelSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorSubsystem;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector.ReefTreeDetectorSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class HighLevelCommandsFactory {

    //private final GroundIntakeCommandFactory groundIntakeCommandFactory;
    //private final TransitWheelsCommandFactory transitWheelsCommandFactory;
    private final OuttakeCommandFactory outtakeCommandFactory;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;
    private final ElevatorCommandFactory elevatorCommandFactory;
    private final AlgaePivotSubsystem algaePivotSubsystem;
    private final AlgaeWheelSubsystem algaeWheelSubsystem;
    private final ReefTreeDetectorSubsystem reefTreeDetectorSubsystem;
    private final DriveSubsystem driveSubsystem;

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
        ReefTreeDetectorSubsystem reefTreeDetectorSubsystem,
        DriveSubsystem driveSubsystem
    ) {
        //this.groundIntakeCommandFactory = groundIntakeCommandFactory;
        //this.transitWheelsCommandFactory = transitWheelsCommandFactory;
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
        this.elevatorCommandFactory = elevatorCommandFactory;
        this.algaeWheelSubsystem = algaeWheelSubsystem;
        this.algaePivotSubsystem = algaePivotSubsystem;
        this.reefTreeDetectorSubsystem = reefTreeDetectorSubsystem;
        this.driveSubsystem = driveSubsystem;
    }
    
    public GroundIntakeCommand createGroundIntakeCommand() {
        return null;//new GroundIntakeCommand(groundIntakeCommandFactory, transitWheelsCommandFactory, outtakeCommandFactory, outtakeCoralSensorsSubsystem, transitCoralSensorSubsystem, elevatorCommandFactory);
    }
    
    public DetectFunnelCoralCommand createDetectFunnelCoralCommand() {
        return new DetectFunnelCoralCommand(outtakeCommandFactory, outtakeCoralSensorsSubsystem);
    }

    public IntakeAlgaeCommand createIntakeAlgaeFromReefL2Command() {
        return new IntakeAlgaeCommand(
            elevatorCommandFactory.createElevatorToReefIntakeAlgaeLowCommand(), 
            new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kReefTreeIntakePositionRotations),
            new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kReefTreeIntakeVoltage),
            this.elevatorCommandFactory, this.algaePivotSubsystem, this.algaeWheelSubsystem);
    }
    public IntakeAlgaeCommand createIntakeAlgaeFromReefL3Command() {
        return new IntakeAlgaeCommand(elevatorCommandFactory.createElevatorToReefIntakeAlgaeHighCommand(), new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kReefTreeIntakePositionRotations),
            new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kReefTreeIntakeVoltage),
            this.elevatorCommandFactory, this.algaePivotSubsystem, this.algaeWheelSubsystem);    
    }
    
    public IntakeAlgaeCommand createIntakeAlgaeFromGroundCommand() {
        return new IntakeAlgaeCommand(elevatorCommandFactory.createElevatorToGroundIntakeAlgaeCommand(), new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kGroundIntakePositionRotations),
            new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kGroundIntakeVoltage),
            this.elevatorCommandFactory, this.algaePivotSubsystem, this.algaeWheelSubsystem);    
    }

    public IntakeAlgaeCommand createLollypopIntakeCommand() {
        return new IntakeAlgaeCommand(elevatorCommandFactory.createElevatorToGroundIntakeAlgaeCommand(), new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kLollypopIntakePositionRotations),
            new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kGroundIntakeVoltage),
            this.elevatorCommandFactory, this.algaePivotSubsystem, this.algaeWheelSubsystem);    
    }


    public OuttakeAlgaeCommand createOuttakeAlgaeOnBargeCommand() {
        return new OuttakeAlgaeCommand(elevatorCommandFactory.createElevatorToBargeScoreCommand(),
            new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kOuttakeBargePositionRotations), 
            this.elevatorCommandFactory,
            this.algaePivotSubsystem, this.algaeWheelSubsystem);
    }

    public OuttakeAlgaeCommand createOuttakeAlgaeInProcessorCommand() {
        return new OuttakeAlgaeCommand(elevatorCommandFactory.createElevatorToProcessorScoreCommand(),
            new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kOuttakeProcessorPositionRotations), 
            this.elevatorCommandFactory,
            this.algaePivotSubsystem, this.algaeWheelSubsystem);
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

    public AlgaePivotToPositionCommand createAlgaePivotToL2IntakePosition() {
        return new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kReefTreeIntakePositionRotations);
    }

    public AlgaePivotToPositionCommand createAlgaePivotToBargeOuttakePosition() {
        return new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kOuttakeBargePositionRotations);
    }

    public AlgaePivotToPositionCommand createAlgaePivotToDefualtPosition() {
        return new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kDefultPivotPosition);
    }

    public ScrollThanOuttakeCommand createPlaceCoralLeftCommand() {
        return new ScrollThanOuttakeCommand(new ScrollWithReefTreeDetectorCommand("TelopScrollLeft",
                new double[] {-.06, .5, 0}, 
                AutoPositionConstants.AutonScrollConstants.kRotationPIDConstants, 
                5, this.driveSubsystem, this.reefTreeDetectorSubsystem::isReefTreeInFrontOfLeft,
                this.reefTreeDetectorSubsystem::isReefTreeInFrontOfRight), .3,
             outtakeCommandFactory);
    }

    public ScrollThanOuttakeCommand createPlaceCoralRightCommand() {
        return new ScrollThanOuttakeCommand(new ScrollWithReefTreeDetectorCommand("TelopScrollRight",
                new double[] {-0.06, -.5, 0}, 
                AutoPositionConstants.AutonScrollConstants.kRotationPIDConstants, 
                5, this.driveSubsystem, this.reefTreeDetectorSubsystem::isReefTreeInFrontOfRight, 
                this.reefTreeDetectorSubsystem::isReefTreeInFrontOfLeft), .3,
                this.outtakeCommandFactory);
    }

    public ScrollThanOuttakeCommand createPlaceCoralLeftCommand(double waitBeforeOuttake) {
        return new ScrollThanOuttakeCommand(new ScrollWithReefTreeDetectorCommand("TelopScrollLeft",
                new double[] {-.1, 0.5, 0}, 
                AutoPositionConstants.AutonScrollConstants.kRotationPIDConstants, 
                5, this.driveSubsystem, this.reefTreeDetectorSubsystem::isReefTreeInFrontOfLeft,
                this.reefTreeDetectorSubsystem::isReefTreeInFrontOfRight), waitBeforeOuttake,
             outtakeCommandFactory);
    }

    public ScrollThanOuttakeCommand createPlaceCoralRightCommand(double waitBeforeOuttake) {
        return new ScrollThanOuttakeCommand(new ScrollWithReefTreeDetectorCommand("TelopScrollRight",
                new double[] {-0.1, -0.5, 0}, 
                AutoPositionConstants.AutonScrollConstants.kRotationPIDConstants, 
                5, this.driveSubsystem, this.reefTreeDetectorSubsystem::isReefTreeInFrontOfRight, 
                this.reefTreeDetectorSubsystem::isReefTreeInFrontOfLeft), waitBeforeOuttake,
                this.outtakeCommandFactory);
    }



}

