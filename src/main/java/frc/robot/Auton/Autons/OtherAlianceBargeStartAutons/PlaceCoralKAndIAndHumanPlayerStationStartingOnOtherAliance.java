package frc.robot.Auton.Autons.OtherAlianceBargeStartAutons;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Constants.GenericConstants.AutonConstants.WPILibAutonConstants;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.CustomWaitCommand;
import frc.robot.Utils.CommandUtils.ParallelGroupCommand;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PlaceCoralKAndIAndHumanPlayerStationStartingOnOtherAliance {
    public static Command getAuton(
        DriveCommandFactory driveCommandFactory, 
        DriveSubsystem driveSubsystem, 
        ElevatorCommandFactory elevatorCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory,
        HighLevelCommandsFactory highLevelCommandsFactory
    ) {
        driveSubsystem.setRobotStartingPose(AutonPointManager.kOtherAllianceBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        
        autonCommands.add(PlaceCoralKAndIStartingOnOtherAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        
        SequentialGroupCommand sequentialGroupCommand = new SequentialGroupCommand(.1, 3.1, new CustomWaitCommand(.03),
            new WPILibFollowTrajectoryFromPointsCommand("CoralIToHumanPlayer",
            AutonPointManager.kCoralIToHumanPlayer,
            2.5,
            new double[] {1.7, 0, 0},
            new double[] {.5, 0, 0},
            new double[] {6, 0, 0},
            4.4,
            3.5,
            WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
            WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
            new Pose2d(.08, .08, Rotation2d.fromDegrees(5)),
            driveSubsystem));

        ParallelGroupCommand elevatorDownWhileDrive = new ParallelGroupCommand(sequentialGroupCommand,
         elevatorCommandFactory.createElevatorToBasementCommand());

        autonCommands.add(elevatorDownWhileDrive);
        autonCommands.add(new CustomWaitCommand(.25));
        
        SequentialGroupCommand auton = GenerateAuto.generateAuto(3, 5, autonCommands);
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
