package frc.robot.Auton.Autons;

import java.util.ArrayList;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.ChoreoAutonCommands.ChoreoTrajectoryCommandCreator;
import frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPose;
import frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPoseAfterTime;
import frc.robot.Commands.AutonCommands.PathplannerAutonCommands.PathPlannerAutonCreatorFromFile;
import frc.robot.Commands.AutonCommands.PathplannerAutonCommands.PathPlannerAutonCreatorFromPoints;
import frc.robot.Commands.AutonCommands.PathplannerAutonCommands.PathPlannerFollowPath;
import frc.robot.Constants.AutonConstants.WPILibAutonConstants;
import frc.robot.Deprecated.WPILibTrajectoryCommandCreator;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class ExampleAuton{

    public static Command getAuton(AutonPointManager autonPointManager, DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(autonPointManager.kExampleStartPoint);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        //autonCommands.add(ChoreoTrajectoryCommandCreator.createChoreoTrajectoryCommand("ExampleAutonChoreo", driveSubsystem));
        //autonCommands.add(PathPlannerAutonCreatorFromPoints.createAutonCommand(autonPointManager.kExampleAutonPointArray, 10, driveSubsystem));
        //autonCommands.add(new PIDGoToPose(autonPointMan ager.kExampleAutonPoint, driveSubsystem));
        //autonCommands.add(PathPlannerAutonCreatorFromFile.createAutonCommand(autonPointManager.kExampleAutonName,
        //   autonPointManager.kExampleAutonEndPoint, 10, driveSubsystem));
        //autonCommands.add(new WPILibTrajectoryCommandCreator("WPILIBExampleAuton",
        //    Rotation2d.fromDegrees(130),
        //    autonPointManager.kExampleWpilibTrajectoryConfig,
        //    driveSubsystem));
        //autonCommands.add(new WPILibTrajectoryCommandCreator("ExampleAuton", autonPointManager.kExampleAutonPointArray, driveSubsystem));
       

        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
  