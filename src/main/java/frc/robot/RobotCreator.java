package frc.robot;


import frc.robot.Constants.GenericConstants.RobotModeConstants;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotIO;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotIOSparkMax;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotSubsystem;
import frc.robot.Subsystems.AlgaeSystem.AlgaeWheels.AlgaeWheelIO;
import frc.robot.Subsystems.AlgaeSystem.AlgaeWheels.AlgaeWheelIOSparkMax;
import frc.robot.Subsystems.AlgaeSystem.AlgaeWheels.AlgaeWheelSubsystem;
import frc.robot.Subsystems.Climber.ClimberIO;
import frc.robot.Subsystems.Climber.ClimberIOTalonFX;
import frc.robot.Subsystems.Climber.ClimberSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOCANCoderMotionMagic;
import frc.robot.Subsystems.Elevator.ElevatorIOCANCoderMotionMagicPIDCatch;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorIO;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorIODistanceSensor;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorSubsystem;
import frc.robot.Subsystems.GroundIntake.IntakeCoralSensor.IntakeCoralSensorSubsystem;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsSubsystem;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsIO;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsIODistanceSensor;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsIO;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsIOSparkMax;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsSubsystem;
import frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector.ReefTreeDetectorIO;
import frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector.ReefTreeDetectorIODistanceSensor;
import frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector.ReefTreeDetectorSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystemCreator;
import frc.robot.Subsystems.TransitTunnel.TransitCoralSensor.TransitCoralSensorSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsSubsystem;

public class RobotCreator {
    // Decloration of subsystem creators
    private final DriveSubsystemCreator driveSubsystemCreator;

    //private final PivotSubsystem pivotSubsystem;
    //private final IntakeWheelsSubsystem intakeWheelsSubsystem;
    //private final IntakeCoralSensorSubsystem intakeCoralSensorSubsystem;

    //private final TransitWheelsSubsystem transitWheelsSubsystem;
    //private final TransitCoralSensorSubsystem transitCoralSensorSubsystem;

    private final FunnelCoralSensorSubsystem funnelCoralSensorSubsystem;
    private final ReefTreeDetectorSubsystem reefTreeDetectorSubsystem;

    private final ElevatorSubsystem elevatorSubsystem;

    private final OuttakeWheelsSubsystem outtakeWheelsSubsystem;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;

    private ClimberSubsystem climberSubsystem;

    private final AlgaeWheelSubsystem algaeWheelSubsystem;
    private final AlgaePivotSubsystem algaePivotSubsystem;


 
    public RobotCreator() {
        this.driveSubsystemCreator = new DriveSubsystemCreator();

        switch (RobotModeConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                this.driveSubsystemCreator.createTalonFXSwerve();

                //this.pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
                //this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIOSparkMax());
                //this.intakeCoralSensorSubsystem = new IntakeCoralSensorSubsystem(new IntakeCoralSensorIODistanceSensor());

                //this.transitWheelsSubsystem = new TransitWheelsSubsystem(new TransitWheelsIOSparkMax());
                //this.transitCoralSensorSubsystem = new TransitCoralSensorSubsystem(new TransitCoralSensorIODistanceSensor());

                this.funnelCoralSensorSubsystem = new FunnelCoralSensorSubsystem(new FunnelCoralSensorIODistanceSensor());
                this.reefTreeDetectorSubsystem = new ReefTreeDetectorSubsystem(new ReefTreeDetectorIODistanceSensor());
                this.outtakeCoralSensorsSubsystem = new OuttakeCoralSensorsSubsystem(new OuttakeCoralSensorsIODistanceSensor());

                this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOCANCoderMotionMagicPIDCatch(), this.funnelCoralSensorSubsystem::isCoralInFunnel, this.outtakeCoralSensorsSubsystem::isCoralOutOfElevator);

                this.outtakeWheelsSubsystem = new OuttakeWheelsSubsystem(new OuttakeWheelsIOSparkMax());

                this.algaeWheelSubsystem = new AlgaeWheelSubsystem(new AlgaeWheelIOSparkMax());
                this.algaePivotSubsystem = new AlgaePivotSubsystem(new AlgaePivotIOSparkMax());

                this.climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementation
                this.driveSubsystemCreator.createSimSwerve();
                //this.pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
                //this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIOSparkMax());
                //this.intakeCoralSensorSubsystem = new IntakeCoralSensorSubsystem(new IntakeCoralSensorIODistanceSensor());

                //this.transitWheelsSubsystem = new TransitWheelsSubsystem(new TransitWheelsIOSparkMax());
                //this.transitCoralSensorSubsystem = new TransitCoralSensorSubsystem(new TransitCoralSensorIODistanceSensor());

                this.funnelCoralSensorSubsystem = new FunnelCoralSensorSubsystem(new FunnelCoralSensorIODistanceSensor());
                this.reefTreeDetectorSubsystem = new ReefTreeDetectorSubsystem(new ReefTreeDetectorIODistanceSensor());
                this.outtakeCoralSensorsSubsystem = new OuttakeCoralSensorsSubsystem(new OuttakeCoralSensorsIODistanceSensor());
                this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOCANCoderMotionMagic(), this.funnelCoralSensorSubsystem::isCoralInFunnel, this.outtakeCoralSensorsSubsystem::isCoralOutOfElevator);

                this.outtakeWheelsSubsystem = new OuttakeWheelsSubsystem(new OuttakeWheelsIOSparkMax());

                this.algaeWheelSubsystem = new AlgaeWheelSubsystem(new AlgaeWheelIOSparkMax());
                this.algaePivotSubsystem = new AlgaePivotSubsystem(new AlgaePivotIOSparkMax());

                this.climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());


                break;
            case REPLAY:
                // Replayed robot, disable IO implementations
                this.driveSubsystemCreator.createReplaySwerve();
                //this.pivotSubsystem = new PivotSubsystem(new PivotIO() {});
                //this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIO() {});
                //this.intakeCoralSensorSubsystem = new IntakeCoralSensorSubsystem(new IntakeCoralSensorIO() {});

                //this.transitWheelsSubsystem = new TransitWheelsSubsystem(new TransitWheelsIO() {});
                //this.transitCoralSensorSubsystem = new TransitCoralSensorSubsystem(new TransitCoralSensorIO() {});

                this.funnelCoralSensorSubsystem = new FunnelCoralSensorSubsystem(new FunnelCoralSensorIO() {});
                this.reefTreeDetectorSubsystem = new ReefTreeDetectorSubsystem(new ReefTreeDetectorIO() {});

                this.outtakeCoralSensorsSubsystem = new OuttakeCoralSensorsSubsystem(new OuttakeCoralSensorsIO() {});

                this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {}, this.funnelCoralSensorSubsystem::isCoralInFunnel, this.outtakeCoralSensorsSubsystem::isCoralOutOfElevator);
                
                this.outtakeWheelsSubsystem = new OuttakeWheelsSubsystem(new OuttakeWheelsIO() {});

                this.algaeWheelSubsystem = new AlgaeWheelSubsystem(new AlgaeWheelIO() {});
                this.algaePivotSubsystem = new AlgaePivotSubsystem(new AlgaePivotIO() {});

                this.climberSubsystem = new ClimberSubsystem(new ClimberIO() {});


                break;
        default:
            throw new RuntimeException("Invalid Robot Mode. Please set the current mode value in RobotModeConstants");
        }
    }

    public DriveSubsystem getDriveSubsystem() {
        return this.driveSubsystemCreator.getDriveSubsystem();
    }

    public PivotSubsystem getPivotSubsystem() {
        return null;
    }

    public IntakeWheelsSubsystem getIntakeWheelsSubsystem() {
        return null;
    }

    public IntakeCoralSensorSubsystem getIntakeCoralSensorSubsystem() {
        return null;
    }

    public TransitWheelsSubsystem getTransitWheelsSubsystem() {
        return null;
    }

    public TransitCoralSensorSubsystem getTransitCoralSensorSubsystem() {
        return null;//this.transitCoralSensorSubsystem;
    }

    public FunnelCoralSensorSubsystem getFunnelCoralSensorSubsystem() {
        return this.funnelCoralSensorSubsystem;
    }

    public ReefTreeDetectorSubsystem getReefTreeDetectorSubsystem() {
        return this.reefTreeDetectorSubsystem;
    }

    public ElevatorSubsystem getElevatorSubsystem() {
        return this.elevatorSubsystem;
    }

    public OuttakeWheelsSubsystem getOuttakeWheelsSubsystem() {
        return this.outtakeWheelsSubsystem;
    }

    public OuttakeCoralSensorsSubsystem getOuttakeCoralSensorsSubsystem() {
        return this.outtakeCoralSensorsSubsystem;
    }

    public AlgaeWheelSubsystem getAlgaeWheelSubsystem() {
        return this.algaeWheelSubsystem;
    }

    public AlgaePivotSubsystem getAlgaePivotSubsystem() {
        return this.algaePivotSubsystem;
    }

    public ClimberSubsystem getClimberSubsystem() {
        return this.climberSubsystem;
    }

}
