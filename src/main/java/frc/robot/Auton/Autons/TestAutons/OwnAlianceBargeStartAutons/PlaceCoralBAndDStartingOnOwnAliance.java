package frc.robot.Auton.Autons.TestAutons.OwnAlianceBargeStartAutons;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PlaceCoralBAndDStartingOnOwnAliance {
    public static Command getAuton(DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(AutonPointManager.kOwnAllianceBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        
        autonCommands.add(PlaceCoralBAndHumanPlayerStartingOnOwnAliance.getAuton(driveCommandFactory, driveSubsystem));
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("HumanPlayerToCoralD",
        AutonPointManager.kHumanPlayerToCoralD,
        5, driveSubsystem));
        
        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
