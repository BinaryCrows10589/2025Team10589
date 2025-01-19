package frc.robot;


import frc.robot.Constants.RobotModeConstants;
import frc.robot.Constants.GroundIntakeConstants.IntakeCoralSensorConstants;
import frc.robot.Subsystems.GroundIntake.IntakeCoralSensor.IntakeCoralSensorIO;
import frc.robot.Subsystems.GroundIntake.IntakeCoralSensor.IntakeCoralSensorIODistanceSensor;
import frc.robot.Subsystems.GroundIntake.IntakeCoralSensor.IntakeCoralSensorSubsystem;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsIO;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsIOSparkMax;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsSubsystem;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotIO;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotIOTalonFX;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotSubsystem;
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
    private final IntakeCoralSensorSubsystem coralSensorIntakeSubsystem;

    private final TransitWheelsSubsystem transitWheelsSubsystem;
    private final TransitCoralSensorSubsystem coralSensorTransitSubsystem;

 
    public RobotCreator() {
        this.driveSubsystemCreator = new DriveSubsystemCreator();

        switch (RobotModeConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // this.driveSubsystemCreator.createTalonFXSwerve();
                this.driveSubsystemCreator.createTalonFXSwerve();

                this.pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
                this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIOSparkMax());
                this.coralSensorIntakeSubsystem = new IntakeCoralSensorSubsystem(new IntakeCoralSensorIODistanceSensor());

                this.transitWheelsSubsystem = new TransitWheelsSubsystem(new TransitWheelsIOSparkMax());
                this.coralSensorTransitSubsystem = new TransitCoralSensorSubsystem(new TransitCoralSensorIODistanceSensor());

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementation
                this.driveSubsystemCreator.createSimSwerve();
                this.pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
                this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIOSparkMax());
                this.coralSensorIntakeSubsystem = new IntakeCoralSensorSubsystem(new IntakeCoralSensorIODistanceSensor());

                this.transitWheelsSubsystem = new TransitWheelsSubsystem(new TransitWheelsIOSparkMax());
                this.coralSensorTransitSubsystem = new TransitCoralSensorSubsystem(new TransitCoralSensorIODistanceSensor());

                break;
            case REPLAY:
                // Replayed robot, disable IO implementations
                this.driveSubsystemCreator.createReplaySwerve();
                this.pivotSubsystem = new PivotSubsystem(new PivotIO() {});
                this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIO() {});
                this.coralSensorIntakeSubsystem = new IntakeCoralSensorSubsystem(new IntakeCoralSensorIO() {});

                this.transitWheelsSubsystem = new TransitWheelsSubsystem(new TransitWheelsIO() {});
                this.coralSensorTransitSubsystem = new TransitCoralSensorSubsystem(new TransitCoralSensorIO() {});
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

    public IntakeCoralSensorSubsystem getCoralSensorIntakeSubsystem() {
        return this.coralSensorIntakeSubsystem;
    }

    public TransitWheelsSubsystem getTransitWheelsSubsystem() {
        return this.transitWheelsSubsystem;
    }

    public TransitCoralSensorSubsystem getCoralSensorTransitSubsystem() {
        return this.coralSensorTransitSubsystem;
    }


}
