package frc.robot;


import frc.robot.Constants.RobotModeConstants;
import frc.robot.Constants.GroundIntakeConstants.CoralSensorIntakeConstants;
import frc.robot.Subsystems.GroundIntake.CoralSensorIntake.CoralSensorIntakeIO;
import frc.robot.Subsystems.GroundIntake.CoralSensorIntake.CoralSensorIntakeIODistanceSensor;
import frc.robot.Subsystems.GroundIntake.CoralSensorIntake.CoralSensorIntakeSubsystem;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsIO;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsIOSparkMax;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsSubsystem;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotIO;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotIOTalonFX;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystemCreator;

public class RobotCreator {
    // Decloration of subsystem creators
    private final DriveSubsystemCreator driveSubsystemCreator;
    private final PivotSubsystem pivotSubsystem;
    private final IntakeWheelsSubsystem intakeWheelsSubsystem;
    private final CoralSensorIntakeSubsystem coralSensorIntakeSubsystem;
 
    public RobotCreator() {
        this.driveSubsystemCreator = new DriveSubsystemCreator();

        switch (RobotModeConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                //this.driveSubsystemCreator.createTalonFXSwerve();
                this.driveSubsystemCreator.createTalonFXSwerve();
                this.pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
                this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIOSparkMax());
                this.coralSensorIntakeSubsystem = new CoralSensorIntakeSubsystem(new CoralSensorIntakeIODistanceSensor());

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementation
                this.driveSubsystemCreator.createSimSwerve();
                this.pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
                this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIOSparkMax());
                this.coralSensorIntakeSubsystem = new CoralSensorIntakeSubsystem(new CoralSensorIntakeIODistanceSensor());

                break;
            case REPLAY:
                // Replayed robot, disable IO implementations
                this.driveSubsystemCreator.createReplaySwerve();
                this.pivotSubsystem = new PivotSubsystem(new PivotIO() {});
                this.intakeWheelsSubsystem = new IntakeWheelsSubsystem(new IntakeWheelsIO() {});
                this.coralSensorIntakeSubsystem = new CoralSensorIntakeSubsystem(new CoralSensorIntakeIO() {});
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

    public CoralSensorIntakeSubsystem getCoralSensorIntakeSubsystem() {
        return this.coralSensorIntakeSubsystem;
    }


}
