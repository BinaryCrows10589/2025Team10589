package frc.robot.Auton.Autons;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PlaceOneCoralBStartingOnOwnAliance {
    public static Command getAuton(DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(AutonPointManager.kOwnAllianceBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("OwnAllianceBargeStartPositionToPlaceOnCoralB",
        AutonPointManager.kOwnAllianceBargeStartPositionToPlaceOnCoralB,
        5, driveSubsystem));
        // Add Coral Placment Command

        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
