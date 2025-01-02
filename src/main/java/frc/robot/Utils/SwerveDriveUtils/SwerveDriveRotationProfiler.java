package frc.robot.Utils.SwerveDriveUtils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class SwerveDriveRotationProfiler extends Command{
    private DriveSubsystem driveSubsystem;
    private double maxRotationRadsPerSecond;
    private boolean finishedBasedOnSpeed = false;
    private double startTime = 0;
    public SwerveDriveRotationProfiler(DriveSubsystem driveSubsystem, double maxRotatoinRadsPerSecond) {
        this.driveSubsystem = driveSubsystem;
        this.maxRotationRadsPerSecond = maxRotatoinRadsPerSecond;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        this.driveSubsystem.drive(0, 0, this.maxRotationRadsPerSecond + 1.4);
    }

    @Override
    public void end(boolean interrupted) {
        if(this.finishedBasedOnSpeed) {
            Logger.recordOutput("SwerveDrive/SpeedProfile/RotationRadiansPerSecond",
                (this.maxRotationRadsPerSecond/(Timer.getFPGATimestamp() - this.startTime)));
        }
        
        this.driveSubsystem.stop();
        this.startTime = 0;
    }

    @Override
    public boolean isFinished() {
        this.finishedBasedOnSpeed = this.driveSubsystem.getChassisSpeeds().omegaRadiansPerSecond >= (this.maxRotationRadsPerSecond);
        return this.finishedBasedOnSpeed;
    }
}
