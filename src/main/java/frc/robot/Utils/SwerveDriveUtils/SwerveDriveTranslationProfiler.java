package frc.robot.Utils.SwerveDriveUtils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class SwerveDriveTranslationProfiler extends Command{
    private DriveSubsystem driveSubsystem;
    private double maxTranslationMPS;
    private boolean finishedBasedOnSpeed = false;
    private double startTime = 0;
    public SwerveDriveTranslationProfiler(DriveSubsystem driveSubsystem, double maxMetersPerSecond) {
        this.driveSubsystem = driveSubsystem;
        this.maxTranslationMPS = maxMetersPerSecond;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        this.driveSubsystem.drive(this.maxTranslationMPS + .6, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        if(this.finishedBasedOnSpeed) {
            Logger.recordOutput("SwerveDrive/SpeedProfile/TranslationAccelerationMPS",
                (this.maxTranslationMPS/(Timer.getFPGATimestamp() - this.startTime)));  
        }
        
        this.driveSubsystem.stop();
        this.startTime = 0;
    }

    @Override
    public boolean isFinished() {
        this.finishedBasedOnSpeed = this.driveSubsystem.getChassisSpeeds().vxMetersPerSecond >= (this.maxTranslationMPS-.02);
        return this.finishedBasedOnSpeed;
    } 
}
