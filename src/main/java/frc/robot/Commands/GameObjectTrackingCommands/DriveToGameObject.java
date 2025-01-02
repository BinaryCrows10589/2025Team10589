package frc.robot.Commands.GameObjectTrackingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.CommandUtils.Wait;

public class DriveToGameObject extends Command{
    private DriveSubsystem driveSubsystem;
    private Wait hardCutOffTimer;
    
    public DriveToGameObject(double maxTime, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.hardCutOffTimer = new Wait(maxTime);
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        this.hardCutOffTimer.startTimer();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupt) {
        this.driveSubsystem.stop();
    } 

    @Override
    public boolean isFinished() {
        return false;
    }
}
