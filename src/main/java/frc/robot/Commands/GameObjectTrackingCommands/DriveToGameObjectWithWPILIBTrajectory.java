package frc.robot.Commands.GameObjectTrackingCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Subsystems.GameObjectTracking.GameObjectTracker;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class DriveToGameObjectWithWPILIBTrajectory extends Command {
    private DriveSubsystem driveSubsystem;
    private WPILibFollowTrajectoryFromPointsCommand driveToGameObjectCommand;
    private double maxTime;
    // TODO: Wokrs on blue, field mirroring issue on red allience
    public DriveToGameObjectWithWPILIBTrajectory(double maxTime, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.maxTime = maxTime;
        addRequirements(this.driveSubsystem);
    
    }  

    @Override
    public void initialize() {
        Pose2d robotPose = driveSubsystem.getRobotPose();
        double goalX = robotPose.getX() + GameObjectTracker.getTargetDistanceAndHeading()[2];
        double goalY = robotPose.getY() + GameObjectTracker.getTargetDistanceAndHeading()[3];
        double goalRotation = robotPose.getRotation().getRadians() + GameObjectTracker.getTargetDistanceAndHeading()[1];
        AutonPoint goalPose = new AutonPoint(goalX, goalY, goalRotation, false);
        AutonPoint[] pointList = {new AutonPoint(robotPose), goalPose};
        this.driveToGameObjectCommand = new WPILibFollowTrajectoryFromPointsCommand("DriveToGameObject", pointList, this.maxTime, driveSubsystem);
        this.driveToGameObjectCommand.schedule();
    }

    @Override
    public void execute() {
        
    }

    @Override 
    public void end(boolean interrupt) {
        this.driveToGameObjectCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        boolean isFinished = this.driveToGameObjectCommand == null ? false : this.driveToGameObjectCommand.isFinished();
        Logger.recordOutput("DriveToGameObject" + "/Finished", isFinished);
        return isFinished;
    }

}
