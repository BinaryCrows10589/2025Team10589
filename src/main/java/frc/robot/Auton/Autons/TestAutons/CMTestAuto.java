package frc.robot.Auton.Autons.TestAutons;

import java.applet.AudioClip;
import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.AudioConfigs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Auton.Autons.OwnAlianceBargeStartAutons.PlaceCoralBAndDAndHumanPlayerStationStartingOnOwnAliance;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Commands.ElevatorCommands.LiftAfterTimeWhenCoralIsInCommand;
import frc.robot.Constants.GenericConstants.AutonConstants.WPILibAutonConstants;
import frc.robot.Constants.GenericConstants.FieldConstants;
import frc.robot.CrowMotion.CMCommand;
import frc.robot.CrowMotion.UserSide.CMAutonPoint;
import frc.robot.CrowMotion.UserSide.CMEvent;
import frc.robot.CrowMotion.UserSide.CMRotation;
import frc.robot.CrowMotion.UserSide.CMTrajectory;
import frc.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.CommandUtils.CustomWaitCommand;
import frc.robot.Utils.CommandUtils.ParallelGroupCommand;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class CMTestAuto {
     public static Command getAuton(
        DriveSubsystem driveSubsystem
    ) {
        driveSubsystem.setRobotStartingPose(new AutonPoint(0, 0, 0, false));
        
        CMCommand command = new CMCommand(
            new CMTrajectory("TestTraj",
                new CMAutonPoint[] {
                    new CMAutonPoint(0, 0, false),
                    new CMAutonPoint(2, 2, false),
                    new CMAutonPoint(1.5, 3, false)
                }, 
                new CMRotation[] {
                    new CMRotation(10, 1, .25, 1, 5, .5, 10, 1),
                   
                    }, 
                    null,
                    TrajectoryPriority.SPLIT_PROPORTIONALLY,
                    4.4, 4.4, 4.4, 0,
                    true,
                    new double[] {.01, .01}, .04, 50
                ), driveSubsystem);
        return command;
    } 

    public static Supplier<Command> getAutonSupplier(DriveSubsystem driveSubsystem) {
        return () -> getAuton(driveSubsystem);
    }
}
