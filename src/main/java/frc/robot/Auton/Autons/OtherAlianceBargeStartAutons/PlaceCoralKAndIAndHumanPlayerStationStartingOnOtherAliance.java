package frc.robot.Auton.Autons.OtherAlianceBargeStartAutons;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Constants.AutonConstants.WPILibAutonConstants;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PlaceCoralKAndIAndHumanPlayerStationStartingOnOtherAliance {
    public static Command getAuton(DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(AutonPointManager.kOtherAllianceBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        
        autonCommands.add(PlaceCoralKAndIStartingOnOtherAliance.getAuton(driveCommandFactory, driveSubsystem));
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("CoralIToHumanPlayer",
        AutonPointManager.kCoralIToHumanPlayer,
        5,
        new double[] {1.5, 0, 0},
        new double[] {3, 0, 0},
        new double[] {1, 0, 0},
        WPILibAutonConstants.kMaxTranslationalSpeedInMetersPerSecond,
        WPILibAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond,
        WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
        WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
        new Pose2d(.08, .08, Rotation2d.fromDegrees(5)),
        driveSubsystem));
        
        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
