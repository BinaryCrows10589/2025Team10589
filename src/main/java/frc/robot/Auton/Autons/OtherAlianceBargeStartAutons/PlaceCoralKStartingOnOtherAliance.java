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

public class PlaceCoralKStartingOnOtherAliance {
    public static Command getAuton(
        DriveCommandFactory driveCommandFactory, 
        DriveSubsystem driveSubsystem, 
        ElevatorCommandFactory elevatorCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory,
        HighLevelCommandsFactory highLevelCommandsFactory
    ) {
        driveSubsystem.setRobotStartingPose(AutonPointManager.kOtherAllianceBargeStartPosition);
        ArrayList<Command> autonCommands = new ArrayList<>();
        SequentialGroupCommand sequentialGroupCommand = new SequentialGroupCommand(new CustomWaitCommand(.85),
            elevatorCommandFactory.createElevatorToL4Command());
        ParallelGroupCommand elevate = new ParallelGroupCommand(sequentialGroupCommand,
            new WPILibFollowTrajectoryFromPointsCommand("OtherAllianceBargeStartPositionToPlaceOnCoralK",
            AutonPointManager.kOtherAllianceBargeStartPositionToPlaceOnCoralK,
            1.32,
            new double[] {1.3, 0, 0},
            new double[] {1.3, 0, 0},
            new double[] {4, 0, 0},
            WPILibAutonConstants.kMaxTranslationalSpeedInMetersPerSecond,
            4.4,
            WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
            WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
            new Pose2d(.06, .06, Rotation2d.fromDegrees(5)),// WPILibAutonConstants.kPositionTolorence,
            driveSubsystem));
        
        autonCommands.add(elevate);
        autonCommands.add(elevatorCommandFactory.createElevatorToL4Command());
        autonCommands.add(highLevelCommandsFactory.createPlaceCoralRightCommand(.3));
        autonCommands.add(outtakeCommandFactory.createOuttakeCoralCommand());
        autonCommands.add(new CustomWaitCommand(.4, "BeforeDown"));

        SequentialGroupCommand auton = GenerateAuto.generateAuto(1.3, 2.1, autonCommands);
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
