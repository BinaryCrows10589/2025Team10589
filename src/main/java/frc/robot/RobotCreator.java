package frc.robot;


import frc.robot.Constants.RobotModeConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystemCreator;

public class RobotCreator {
    // Decloration of subsystem creators
    private final DriveSubsystemCreator driveSubsystemCreator;
 
    public RobotCreator() {
        this.driveSubsystemCreator = new DriveSubsystemCreator();

        switch (RobotModeConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                //this.driveSubsystemCreator.createTalonFXSwerve();
                this.driveSubsystemCreator.createTalonFXSwerve();
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementation
                this.driveSubsystemCreator.createSimSwerve();
                break;
            case REPLAY:
                // Replayed robot, disable IO implementations
                this.driveSubsystemCreator.createReplaySwerve();
                break;
        default:
            throw new RuntimeException("Invalid Robot Mode. Please set the current mode value in RobotModeConstants");
        }
    }

    public DriveSubsystem getDriveSubsystem() {
        return this.driveSubsystemCreator.getDriveSubsystem();
    }
}
