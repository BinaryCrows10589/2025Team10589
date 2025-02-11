package frc.robot.Utils.SwerveDriveUtils;

import frc.robot.Commands.SwerveDriveCommands.FieldOrientedDriveCommand;
import frc.robot.Constants.MechanismConstants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Subsystems.SwerveDrive.Gyro.GyroIO;
import frc.robot.Subsystems.SwerveDrive.Gyro.GyroIOPigeon2;
import frc.robot.Subsystems.SwerveDrive.Gyro.GyroIOSim;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIO;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIOSim;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIOTalonFX;
import frc.robot.Utils.JoystickUtils.ControllerInterface;

public class SwerveDriveSetupUtils {
    public static DriveSubsystem createTalonFXSwerve() {
        return new DriveSubsystem(
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kBackRightModuleName),
            new GyroIOPigeon2());         
    }

    public static DriveSubsystem createSimSwerve() {
        return new DriveSubsystem(
            new SwerveModuleIOSim(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kBackRightModuleName),
            new GyroIOSim());         
    }

    public static DriveSubsystem createReplaySwerve() {
        return new DriveSubsystem(
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new GyroIO() {});         
    }

    public static void createFieldOrientedDriveCommand(DriveSubsystem driveSubsystem,
    ElevatorSubsystem elevatorSubsystem,
        ControllerInterface driverController) {
        FieldOrientedDriveCommand fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
        driveSubsystem, 
        elevatorSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX());
        
        driveSubsystem.setDefaultCommand(fieldOrientedDriveCommand);
    }
}
