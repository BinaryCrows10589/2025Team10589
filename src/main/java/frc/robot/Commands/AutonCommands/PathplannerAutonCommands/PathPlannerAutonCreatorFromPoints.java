package frc.robot.Commands.AutonCommands.PathplannerAutonCommands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class PathPlannerAutonCreatorFromPoints {

    public static Command createAutonCommand(AutonPoint[] points, double maxTime, DriveSubsystem driveSubsystem) {
        AutonPoint startPoint = points[0];
        AutonPoint endPoint = points[points.length-1];
        Pose2d realStartPoint = new Pose2d(startPoint.getAutonPoint().getTranslation(), new Rotation2d());
        Pose2d realEndPoint = new Pose2d(endPoint.getAutonPoint().getTranslation(), new Rotation2d());
        
        ArrayList<Pose2d> pointsAsPose2d = new ArrayList<>();
        pointsAsPose2d.add(realStartPoint);
        for(int i = 1; i < points.length-1; i++) {
            Pose2d pointAsPose2d = points[i].getAutonPoint();
            Pose2d modifedPoint = new Pose2d(pointAsPose2d.getTranslation(), new Rotation2d());
            pointsAsPose2d.add(modifedPoint);
        }
        pointsAsPose2d.add(realEndPoint);
        /* 
        List<Waypoint> bezierPoints = PathPlannerPath.bezierFromPoses(
                realStartPoint,
                realEndPoint
        );
        
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(PathPlannerAutonConstants.kMaxModuleSpeedMetersPerSecond,
                    PathPlannerAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond,
                    PathPlannerAutonConstants.kMaxRotationalSpeedInDegrees,
                    PathPlannerAutonConstants.kMaxRotationalAccelerationInDegrees), 
                new GoalEndState(0.0, Rotation2d.fromDegrees(endPoint.getAutonPoint().getRotation().getDegrees()))
        );

        path.preventFlipping = true;

        return new PathPlannerFollowPath(path, endPoint.getAutonPoint(), maxTime, driveSubsystem);
        */
        return new Command() {
            
        };
    }
}
