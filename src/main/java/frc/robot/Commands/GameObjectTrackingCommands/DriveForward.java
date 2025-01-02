package frc.robot.Commands.GameObjectTrackingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.CommandUtils.Wait;

public class DriveForward extends Command{
    private final DriveSubsystem driveSubsystem;
    private final double speedMetersPerSecond;
    private final Wait hardCutOffTimer;
    
    public DriveForward(double speedMetersPerSecond, double maxTime, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.hardCutOffTimer = new Wait(maxTime);
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        this.hardCutOffTimer.startTimer();
    }

    @Override
    public void execute() {
        this.driveSubsystem.drive(this.speedMetersPerSecond, 0, 0, false);
    }

    @Override
    public void end(boolean interrupt) {
        this.driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return this.hardCutOffTimer.hasTimePassed();
    }
}
