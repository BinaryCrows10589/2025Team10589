package frc.robot;


import frc.robot.Constants.RobotModeConstants;
import frc.robot.Constants.GroundIntakeConstants.IntakeCoralSensorConstants;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorIO;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorIODistanceSensor;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorSubsystem;
import frc.robot.Subsystems.GroundIntake.IntakeCoralSensor.IntakeCoralSensorIO;
import frc.robot.Subsystems.GroundIntake.IntakeCoralSensor.IntakeCoralSensorIODistanceSensor;
import frc.robot.Subsystems.GroundIntake.IntakeCoralSensor.IntakeCoralSensorSubsystem;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsIO;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsIOSparkMax;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsSubsystem;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotIO;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotIOTalonFX;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotSubsystem;
import frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector.ReefTreeDetectorIO;
import frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector.ReefTreeDetectorIODistanceSensor;
import frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector.ReefTreeDetectorSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystemCreator;
import frc.robot.Subsystems.TransitTunnel.TransitCoralSensor.TransitCoralSensorIO;
import frc.robot.Subsystems.TransitTunnel.TransitCoralSensor.TransitCoralSensorIODistanceSensor;
import frc.robot.Subsystems.TransitTunnel.TransitCoralSensor.TransitCoralSensorSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsIO;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsIOSparkMax;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsSubsystem;

public class RobotCreator {
    // Decloration of subsystem creators
    private final DriveSubsystemCreator driveSubsystemCreator;

    private final PivotSubsystem pivotSubsystem;
    private final IntakeWheelsSubsystem intakeWheelsSubsystem;
    private final IntakeCoralSensorSubsystem intakeCoralSensorSubsystem;

    private final TransitWheelsSubsystem transitWheelsSubsystem;
    private final TransitCoralSensorSubsystem transitCoralSensorSubsystem;

    private final FunnelCoralSensorSubsystem funnelCoralSensorSubsystem;
    private final ReefTreeDetectorSubsystem reefTreeDetectorSubsystem;

 
    public RobotCreator() {
        this.driveSubsystemCreator = new DriveSubsystemCreator();

        switch (RobotModeConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // this.driveSubsystemCreator.createTalonFXSwerve();
                this.driveSubsystemCreator.createTalonFXSwerve();

                this.pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
                this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIOSparkMax());
                this.intakeCoralSensorSubsystem = new IntakeCoralSensorSubsystem(new IntakeCoralSensorIODistanceSensor());

                this.transitWheelsSubsystem = new TransitWheelsSubsystem(new TransitWheelsIOSparkMax());
                this.transitCoralSensorSubsystem = new TransitCoralSensorSubsystem(new TransitCoralSensorIODistanceSensor());

                this.funnelCoralSensorSubsystem = new FunnelCoralSensorSubsystem(new FunnelCoralSensorIODistanceSensor());
                this.reefTreeDetectorSubsystem = new ReefTreeDetectorSubsystem(new ReefTreeDetectorIODistanceSensor());

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementation
                this.driveSubsystemCreator.createSimSwerve();
                this.pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
                this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIOSparkMax());
                this.intakeCoralSensorSubsystem = new IntakeCoralSensorSubsystem(new IntakeCoralSensorIODistanceSensor());

                this.transitWheelsSubsystem = new TransitWheelsSubsystem(new TransitWheelsIOSparkMax());
                this.transitCoralSensorSubsystem = new TransitCoralSensorSubsystem(new TransitCoralSensorIODistanceSensor());

                this.funnelCoralSensorSubsystem = new FunnelCoralSensorSubsystem(new FunnelCoralSensorIODistanceSensor());
                this.reefTreeDetectorSubsystem = new ReefTreeDetectorSubsystem(new ReefTreeDetectorIODistanceSensor());

                break;
            case REPLAY:
                // Replayed robot, disable IO implementations
                this.driveSubsystemCreator.createReplaySwerve();
                this.pivotSubsystem = new PivotSubsystem(new PivotIO() {});
                this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIO() {});
                this.intakeCoralSensorSubsystem = new IntakeCoralSensorSubsystem(new IntakeCoralSensorIO() {});

                this.transitWheelsSubsystem = new TransitWheelsSubsystem(new TransitWheelsIO() {});
                this.transitCoralSensorSubsystem = new TransitCoralSensorSubsystem(new TransitCoralSensorIO() {});

                this.funnelCoralSensorSubsystem = new FunnelCoralSensorSubsystem(new FunnelCoralSensorIO() {});
                this.reefTreeDetectorSubsystem = new ReefTreeDetectorSubsystem(new ReefTreeDetectorIO() {});


                break;
        default:
            throw new RuntimeException("Invalid Robot Mode. Please set the current mode value in RobotModeConstants");
        }
    }

    public DriveSubsystem getDriveSubsystem() {
        return this.driveSubsystemCreator.getDriveSubsystem();
    }

    public PivotSubsystem getPivotSubsystem() {
        return this.pivotSubsystem;
    }

    public IntakeWheelsSubsystem getIntakeWheelsSubsystem() {
        return this.intakeWheelsSubsystem;
    }

    public IntakeCoralSensorSubsystem getIntakeCoralSensorSubsystem() {
        return this.intakeCoralSensorSubsystem;
    }

    public TransitWheelsSubsystem getTransitWheelsSubsystem() {
        return this.transitWheelsSubsystem;
    }

    public TransitCoralSensorSubsystem getTransitCoralSensorSubsystem() {
        return this.transitCoralSensorSubsystem;
    }

    public FunnelCoralSensorSubsystem getFunnelCoralSensorSubsystem() {
        return this.funnelCoralSensorSubsystem;
    }

    public ReefTreeDetectorSubsystem getReefTreeDetectorSubsystem() {
        return this.reefTreeDetectorSubsystem;
    }

}
