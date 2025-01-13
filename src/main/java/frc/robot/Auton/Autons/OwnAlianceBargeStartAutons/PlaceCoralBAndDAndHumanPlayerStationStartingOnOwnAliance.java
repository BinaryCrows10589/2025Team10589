package frc.robot.Auton.Autons.OwnAlianceBargeStartAutons;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Constants.AutonConstants.WPILibAutonConstants;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PlaceCoralBAndDAndHumanPlayerStationStartingOnOwnAliance {
    public static Command getAuton(DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(AutonPointManager.kOwnAllianceBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        
        autonCommands.add(PlaceCoralBAndDStartingOnOwnAliance.getAuton(driveCommandFactory, driveSubsystem));
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("CoralDToHumanPlayer",
        AutonPointManager.kCoralDToHumanPlayer,
        5,
        new double[] {.5, 0, 0},
        new double[] {.5, 0, 0},
        new double[] {.5, 0, 0},
        WPILibAutonConstants.kMaxTranslationalSpeedInMetersPerSecond,
        WPILibAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond,
        WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
        WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
        WPILibAutonConstants.kPositionTolorence,
        driveSubsystem));
        
        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
