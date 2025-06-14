package frc.robot.Auton.Autons.CenterBargeStartAutons;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Commands.SwerveDriveCommands.DriveForwardCommand;
import frc.robot.Constants.GenericConstants.AutonConstants.WPILibAutonConstants;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.CustomWaitCommand;
import frc.robot.Utils.CommandUtils.EndCommandAfterWait;
import frc.robot.Utils.CommandUtils.ParallelGroupCommand;
import frc.robot.Utils.CommandUtils.ParallelRaceGroupCommand;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PlaceAlgaeNet {
    public static Command getAuton(
        DriveCommandFactory driveCommandFactory, 
        DriveSubsystem driveSubsystem, 
        ElevatorCommandFactory elevatorCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory,
        HighLevelCommandsFactory highLevelCommandsFactory
    ) {
        driveSubsystem.setRobotStartingPose(AutonPointManager.kCenterBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();

        autonCommands.add(PlaceCoralLStartingCenterBargeAfterGrabbingAlgae.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));//PlaceCoralLStartingCenterBargeAfterGrabbingAlgae.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        
        SequentialGroupCommand sequentialGroupCommand = new SequentialGroupCommand(.1, 3.1, new CustomWaitCommand(.03),
        new WPILibFollowTrajectoryFromPointsCommand("CenterBargeStartPositionPlaceAlgaePosition",
            AutonPointManager.kCenterBargeStartPositionPlaceAlgaePosition,
            3,
            new double[] {3, 0, 0},
            new double[] {6, 0, 0},
            new double[] {12, 0, 0},
            3.6,
            3.75,
            WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
            WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
            new Pose2d(.25, .25, Rotation2d.fromDegrees(10)),
            driveSubsystem));
        ParallelGroupCommand elevatorDownWhileDrive = new ParallelGroupCommand(sequentialGroupCommand,
        elevatorCommandFactory.createElevatorToBasementCommand());

        autonCommands.add(elevatorDownWhileDrive);
        
        autonCommands.add(new EndCommandAfterWait(highLevelCommandsFactory.createOuttakeAlgaeOnBargeCommand(), 1));
        autonCommands.add(highLevelCommandsFactory.createAlgaePivotToBargeOuttakePosition());
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("CenterBargeStartPositionPlaceAlgaePositionSlowDriveIn",
            AutonPointManager.kCenterBargeStartPositionPlaceSlowDriveInAlgaePosition,
            2,
            new double[] {6, 0, 0},
            new double[] {6, 0, 0},
            new double[] {6, 0, 0},
            .75,
            .75,
            WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
            WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
            WPILibAutonConstants.kPositionTolorence,
            driveSubsystem));
        autonCommands.add(new EndCommandAfterWait(highLevelCommandsFactory.createOutakeWheelsAlgaeBargeCommand(), .25));
        autonCommands.add(highLevelCommandsFactory.createAlgaePivotToDefualtPosition());
        autonCommands.add(new EndCommandAfterWait(new DriveForwardCommand(driveSubsystem, -.75, 0, 0), 3));
        //autonCommands.add(elevatorCommandFactory.createElevatorToBasementCommand());

        SequentialGroupCommand auton = GenerateAuto.generateAuto(5, 15, autonCommands);
        return auton;
    } 

    public static Supplier<Command> getAutonSupplier(
        DriveCommandFactory driveCommandFactory, 
        DriveSubsystem driveSubsystem, 
        ElevatorCommandFactory elevatorCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory,
        HighLevelCommandsFactory highLevelCommandsFactory
    ) {
        return () -> getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory);
    }
}
