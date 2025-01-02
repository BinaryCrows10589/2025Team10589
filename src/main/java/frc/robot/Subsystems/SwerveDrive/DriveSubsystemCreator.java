package frc.robot.Subsystems.SwerveDrive;

import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Subsystems.SwerveDrive.Gyro.GyroIO;
import frc.robot.Subsystems.SwerveDrive.Gyro.GyroIOPigeon2;
import frc.robot.Subsystems.SwerveDrive.Gyro.GyroIOSim;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIO;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIOSim;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIOSparkMax;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIOTalonFX;

public class DriveSubsystemCreator {
    private DriveSubsystem driveSubsystem;

     public DriveSubsystem getDriveSubsystem() {
        return this.driveSubsystem;
    }

    public void createSparkMaxSwerve() {
        this.driveSubsystem = new DriveSubsystem(
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kBackRightModuleName),
            new GyroIOPigeon2());         
    }

    public void createTalonFXSwerve() {
        this.driveSubsystem = new DriveSubsystem(
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kBackRightModuleName),
            new GyroIOPigeon2());         
    }

    public void createSimSwerve() {
        this.driveSubsystem = new DriveSubsystem(
            new SwerveModuleIOSim(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kBackRightModuleName),
            new GyroIOSim());         
    }

    public void createReplaySwerve() {
        this.driveSubsystem = new DriveSubsystem(
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new GyroIO() {});         
    }
}
