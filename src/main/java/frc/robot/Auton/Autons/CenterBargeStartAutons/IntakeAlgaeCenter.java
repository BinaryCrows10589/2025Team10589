package frc.robot.Auton.Autons.CenterBargeStartAutons;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
import frc.robot.Utils.CommandUtils.CustomWaitCommand;
import frc.robot.Utils.CommandUtils.ParallelGroupCommand;
import frc.robot.Utils.CommandUtils.ParallelRaceGroupCommand;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class IntakeAlgaeCenter {
    public static Command getAuton(
        DriveCommandFactory driveCommandFactory, 
        DriveSubsystem driveSubsystem, 
        ElevatorCommandFactory elevatorCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory,
        HighLevelCommandsFactory highLevelCommandsFactory
    ) {
        driveSubsystem.setRobotStartingPose(AutonPointManager.kCenterBargeStartPosition);
        
        ArrayList<Command> autonCommands = new ArrayList<>();

        ParallelDeadlineGroup intakeForTime =  new ParallelDeadlineGroup(new WaitCommand(4),
          highLevelCommandsFactory.createIntakeAlgaeFromReefL2Command());
   
        autonCommands.add(intakeForTime);
        /*
        ParallelGroupCommand driveWhileIntaking = new ParallelGroupCommand(
            new WPILibFollowTrajectoryFromPointsCommand("CenterBargeStartPositionToIntakeCenterAlgae",
            AutonPointManager.kCenterBargeStartPositionToIntakeCenterAlgae,
            5,
            new double[] {6, 0, 0},
            new double[] {2, 0, 0},
            new double[] {12, 0, 0},
            2,
            2,
            WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
            WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
            WPILibAutonConstants.kPositionTolorence,
            driveSubsystem),
            new SequentialGroupCommand(new CustomWaitCommand(.5),
                intakeForTime,
                highLevelCommandsFactory.createAlgaePivotToL2IntakePosition()
            ));
        autonCommands.add(driveWhileIntaking);
         */
        /* 
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("CenterBargeStartPositionBackUpFromIntakeCenterAlgae",
        AutonPointManager.kCenterBargeStartPositionBackUpFromIntakeCenterAlgae,
        5,
        new double[] {6, 0, 0},
        new double[] {2, 0, 0},
        new double[] {12, 0, 0},
        WPILibAutonConstants.kMaxTranslationalSpeedInMetersPerSecond,
        WPILibAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond,
        WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
        WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond,
        WPILibAutonConstants.kPositionTolorence,
        driveSubsystem));
        autonCommands.add(highLevelCommandsFactory.createAlgaePivotToDefualtPosition());
        autonCommands.add(elevatorCommandFactory.createElevatorToBasementCommand());
*/
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
