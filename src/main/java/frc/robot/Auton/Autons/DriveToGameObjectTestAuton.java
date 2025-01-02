package frc.robot.Auton.Autons;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Commands.GameObjectTrackingCommands.DriveForward;
import frc.robot.Commands.GameObjectTrackingCommands.RotateToFaceGameObject;
import frc.robot.Commands.GameObjectTrackingCommands.RotateToFaceGameObjectWhileDriving;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.ParallelGroupCommand;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class DriveToGameObjectTestAuton{

    public static Command getAuton(AutonPointManager autonPointManager, DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(autonPointManager.kDriveToGameObjectTestAutonStartPoint);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        autonCommands.add(new RotateToFaceGameObjectWhileDriving(3, 2, 12, driveSubsystem));        
        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
        
    } 
}
  