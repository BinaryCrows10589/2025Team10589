package frc.robot.CrowMotion;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CrowMotion.UserSide.CMTrajectory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class CMCommand extends Command{
    private CMTrajectory trajectory;
    
    public CMCommand(CMTrajectory trajectory, DriveSubsystem driveSubsystem) {
        this.trajectory = trajectory;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.trajectory.init();
    }

    @Override
    public void execute() {
        this.trajectory.runTrajectoryFrame();
    }

    @Override
    public void end(boolean interupt) {
        
    }

    @Override
    public boolean isFinished() {
        return this.trajectory.isCompleted();
    }


}
