package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants.VisionConstants;

public class DisableVisionCommand extends Command{
    private boolean shouldEnd = false;
    
    public DisableVisionCommand() {
        this.shouldEnd = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  
        VisionConstants.updateVision = false;
        this.shouldEnd = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shouldEnd = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shouldEnd;
    }
}
