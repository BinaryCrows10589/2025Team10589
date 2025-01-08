package frc.robot.Auton.Autons.TestAutons;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class ExampleAuton{

    public static Command getAuton(AutonPointManager autonPointManager, DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        //driveSubsystem.setRobotPose(autonPointManager.kExampleStartPoint);
        
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
  