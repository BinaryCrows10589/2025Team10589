package frc.robot.Subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.SwerveDriveCommands.FieldOrientedDriveCommand;
import frc.robot.Commands.SwerveDriveCommands.LockSwerves;
import frc.robot.Utils.JoystickUtils.ControllerInterface;
import frc.robot.Utils.SwerveDriveUtils.SwerveDriveRotationProfiler;
import frc.robot.Utils.SwerveDriveUtils.SwerveDriveTranslationProfiler;

public class DriveCommandFactory {
    // All Subsystems needed for every command
    DriveSubsystem driveSubsystem;

    // All other dependencies for every command that uses primarly DriveSubsystem
    ControllerInterface driverController;

    public DriveCommandFactory(DriveSubsystem driveSubsystem, ControllerInterface driverController) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
    }

    public FieldOrientedDriveCommand createFieldOrientedDriveCommand() {
        return new FieldOrientedDriveCommand(
        this.driveSubsystem, 
        () -> -this.driverController.getLeftY(),
        () -> -this.driverController.getLeftX(),
        () -> -this.driverController.getRightX());
    }

    public LockSwerves createLockSwervesCommand() {
        return new LockSwerves(this.driveSubsystem);
    }

    public SwerveDriveTranslationProfiler createSwerveDriveTranslationProfiler() {
        return new SwerveDriveTranslationProfiler(this.driveSubsystem, 3.4);
    }

    public SwerveDriveRotationProfiler createSwerveDriveRotationProfiler() {
        return new SwerveDriveRotationProfiler(driveSubsystem, 7.5);//9.892);
    }

    public Command createResetOdometryCommand() {
        return Commands.runOnce(() -> this.driveSubsystem.resetRobotPose());
    }

}
