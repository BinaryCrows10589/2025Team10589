package frc.robot.Auton.Autons.CenterBargeStartAutons;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Constants.GenericConstants.AutonConstants.WPILibAutonConstants;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PlaceCoralLStartingCenterBarge {
    public static Command getAuton(
        DriveCommandFactory driveCommandFactory, 
        DriveSubsystem driveSubsystem, 
        ElevatorCommandFactory elevatorCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory
    ) {
        driveSubsystem.setRobotPose(AutonPointManager.kCenterBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("CenterBargeStartPositionToPlaceOnCoralL",
        AutonPointManager.kCenterBargeStartPositionToPlaceOnCoralL,
        5,
        new double[] {.15, 0, 0},
        new double[] {.15, 0, 0},
        new double[] {.6, 0, 0},
        WPILibAutonConstants.kMaxTranslationalSpeedInMetersPerSecond,
        WPILibAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond,
        WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
        WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
        WPILibAutonConstants.kPositionTolorence,
        driveSubsystem));
        autonCommands.add(elevatorCommandFactory.createElevatorToL4Command());
        autonCommands.add(outtakeCommandFactory.createOuttakeCoralCommand());
        autonCommands.add(elevatorCommandFactory.createElevatorToBasementCommand());

        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
