package frc.robot.Commands.AutonCommands.PathplannerAutonCommands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotModeConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class PathPlannerAutonCreatorFromFile {

    public static Command createAutonCommand(String autonName, AutonPoint endPoint, double maxTime, DriveSubsystem driveSubsystem) {
        String allianceColor = RobotModeConstants.isBlueAlliance ? "" : "_red";
        return new PathPlannerFollowPath(AutoBuilder.buildAuto(autonName + allianceColor), endPoint.getAutonPoint(), maxTime, driveSubsystem);
    }
}
