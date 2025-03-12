package frc.robot.Auton.Autons.OwnAlianceBargeStartAutons;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Commands.ElevatorCommands.LiftAfterTimeWhenCoralIsInCommand;
import frc.robot.Constants.GenericConstants.AutonConstants.WPILibAutonConstants;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.CustomWaitCommand;
import frc.robot.Utils.CommandUtils.ParallelGroupCommand;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PlaceCoralBAndDStartingOnOwnAliance {
    public static Command getAuton(
        DriveCommandFactory driveCommandFactory, 
        DriveSubsystem driveSubsystem, 
        ElevatorCommandFactory elevatorCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory,
        HighLevelCommandsFactory highLevelCommandsFactory
    ) {
        driveSubsystem.setRobotStartingPose(AutonPointManager.kOwnAllianceBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        
        autonCommands.add(PlaceCoralBAndHumanPlayerStartingOnOwnAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        
        ParallelGroupCommand elevateWhileDriving = new ParallelGroupCommand(
            new WPILibFollowTrajectoryFromPointsCommand("HumanPlayerToCoralD",
            AutonPointManager.kHumanPlayerToCoralD,
                1.7,
                new double[] {2, 0, 0},
                new double[] {6, 0, 0},
                new double[] {6, 0, 0},
                WPILibAutonConstants.kMaxTranslationalSpeedInMetersPerSecond,
                WPILibAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond,
                WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
                WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
                WPILibAutonConstants.kPositionTolorence,
                driveSubsystem), new LiftAfterTimeWhenCoralIsInCommand(elevatorCommandFactory.createElevatorToL4Command(),
                1.4)
        );
        autonCommands.add(elevateWhileDriving);
        autonCommands.add(elevatorCommandFactory.createElevatorToL4Command());
        autonCommands.add(highLevelCommandsFactory.createPlaceCoralRightCommand(.3));
        autonCommands.add(outtakeCommandFactory.createOuttakeCoralCommand());
        //autonCommands.add(outtakeCommandFactory.createOuttakeCoralCommand());
        //autonCommands.add(elevatorCommandFactory.createElevatorToBasementCommand());

        SequentialGroupCommand auton = GenerateAuto.generateAuto(1.2, 5, autonCommands);

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
