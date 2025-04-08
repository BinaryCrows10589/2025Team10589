package frc.robot.Auton.Autons.CenterBargeStartAutons;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PlaceCoralLStartingCenterBargeAfterGrabbingAlgae {
    public static Command getAuton(
        DriveCommandFactory driveCommandFactory, 
        DriveSubsystem driveSubsystem, 
        ElevatorCommandFactory elevatorCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory,
        HighLevelCommandsFactory highLevelCommandsFactory
    ) {
        driveSubsystem.setRobotStartingPose(AutonPointManager.kCenterBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        autonCommands.add(IntakeAlgaeCenter.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));

        autonCommands.add(new EndCommandAfterWait(new DriveForwardCommand(driveSubsystem, -2, 0, 0), .6));


        autonCommands.add(elevatorCommandFactory.createElevatorToL4Command());
        autonCommands.add(highLevelCommandsFactory.createPlaceCoralLeftCommand(.3));
        autonCommands.add(outtakeCommandFactory.createOuttakeCoralCommand());

        SequentialGroupCommand auton = GenerateAuto.generateAuto(1.4,5, autonCommands);
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
