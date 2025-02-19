package frc.robot.Auton.Autons.OtherAlianceBargeStartAutons;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Constants.GenericConstants.AutonConstants.WPILibAutonConstants;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PlaceCoralKStartingOnOtherAliance {
    public static Command getAuton(
        DriveCommandFactory driveCommandFactory, 
        DriveSubsystem driveSubsystem, 
        ElevatorCommandFactory elevatorCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory,
        HighLevelCommandsFactory highLevelCommandsFactory
    ) {
        driveSubsystem.setRobotPose(AutonPointManager.kOtherAllianceBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("OtherAllianceBargeStartPositionToPlaceOnCoralK",
        AutonPointManager.kOtherAllianceBargeStartPositionToPlaceOnCoralK,
        5,
        new double[] {1.3, 0, 0},
        new double[] {.75, 0, 0},
        new double[] {.75, 0, 0},
        WPILibAutonConstants.kMaxTranslationalSpeedInMetersPerSecond,
        WPILibAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond,
        WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
        WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
        new Pose2d(.06, .06, Rotation2d.fromDegrees(5)),// WPILibAutonConstants.kPositionTolorence,
        driveSubsystem));
        autonCommands.add(elevatorCommandFactory.createElevatorToL4Command());
        autonCommands.add(highLevelCommandsFactory.createPlaceCoralLeftCommand());
        autonCommands.add(new WaitCommand(1));
        //autonCommands.add(outtakeCommandFactory.createOuttakeCoralCommand());
        autonCommands.add(elevatorCommandFactory.createElevatorToBasementCommand());

        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
