package frc.robot.Auton;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.Autons.CenterBargeStartAutons.PlaceCoralAStartingCenterBarge;
import frc.robot.Auton.Autons.CenterBargeStartAutons.PlaceCoralLStartingCenterBarge;
import frc.robot.Auton.Autons.OtherAlianceBargeStartAutons.PlaceCoralKAndHumanPlayerStartingOnOtherAliance;
import frc.robot.Auton.Autons.OtherAlianceBargeStartAutons.PlaceCoralKAndIAndHStartingOnOtherAliance;
import frc.robot.Auton.Autons.OtherAlianceBargeStartAutons.PlaceCoralKAndIAndHumanPlayerStationStartingOnOtherAliance;
import frc.robot.Auton.Autons.OtherAlianceBargeStartAutons.PlaceCoralKAndIStartingOnOtherAliance;
import frc.robot.Auton.Autons.OtherAlianceBargeStartAutons.PlaceCoralKStartingOnOtherAliance;
import frc.robot.Auton.Autons.OwnAlianceBargeStartAutons.PlaceCoralBAndDAndEStartingOnOwnAliance;
import frc.robot.Auton.Autons.OwnAlianceBargeStartAutons.PlaceCoralBAndDAndHumanPlayerStationStartingOnOwnAliance;
import frc.robot.Auton.Autons.OwnAlianceBargeStartAutons.PlaceCoralBAndDStartingOnOwnAliance;
import frc.robot.Auton.Autons.OwnAlianceBargeStartAutons.PlaceCoralBAndHumanPlayerStartingOnOwnAliance;
import frc.robot.Auton.Autons.OwnAlianceBargeStartAutons.PlaceCoralBStartingOnOwnAliance;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.CommandUtils.CustomWaitCommand;

public class AutonManager {
    // Decloration of auton names
    private final String placeCoralBStartingOnOwnAlianceAuton = "PlaceCoralBStartingOnOwnAliance";
    private final String placeCoralBAndHumanPlayerStartingOnOwnAlianceAuton = "PlaceCoralBAndHumanPlayerStartingOnOwnAliance";
    private final String placeCoralBAndDStartingOnOwnAlianceAuton = "PlaceCoralBAndDStartingOnOwnAliance";
    private final String placeCoralBAndDAndHumanPlayerStationStartingOnOwnAlianceAuton = "PlaceCoralBAndDAndHumanPlayerStartingOnOwnAliance";
    private final String placeCoralBAndDAndEStartingOnOwnAlianceAuton = "PlaceCoralBAndDAndEStartingOnOwnAliance";

    private final String placeCoralAStartingCenterBarge = "CenterBargeStartPositionToPlaceOnCoralA";
    private final String placeCoralLStartingCenterBarge = "CenterBargeStartPositionToPlaceOnCoralL";
    
    private final String placeCoralKStartingOnOtherAlianceAuton = "PlaceCoralKStartingOnOtherAliance";
    private final String placeCoralKAndHumanPlayerStartingOnOtherAlianceAuton = "PlaceCoralKAndHumanPlayerStartingOnOtherAliance";
    private final String placeCoralKAndIStartingOnOtherAlianceAuton = "PlaceCoralKAndIStartingOnOtherAliance";
    private final String placeCoralKAndIAndHumanPlayerStationStartingOnOtherAlianceAuton = "PlaceCoralKAndIAndHumanPlayerStartingOnOtherAliance";
    private final String placeCoralKAndIAndHStartingOnOtherAlianceAuton = "PlaceCoralKAndIAndHStartingOnOtherAliance";

    // Decloration of auton chooser
    private LoggedDashboardChooser<String> autonChooser;

    // Decloration of all auton dependencies
    private DriveCommandFactory driveCommandFactory;
    private DriveSubsystem driveSubsystem;
    private ElevatorCommandFactory elevatorCommandFactory;
    private OuttakeCommandFactory outtakeCommandFactory;
    private HighLevelCommandsFactory highLevelCommandsFactory;

    private HashMap<String, Supplier<Command>> autonomousCommands = new HashMap<>();
    private Command selectedAutonomousCommand = new Command() {}; // Default to empty

    public AutonManager(DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem, ElevatorCommandFactory elevatorCommandFactory, OuttakeCommandFactory outtakeCommandFactory, HighLevelCommandsFactory highLevelCommandsFactory) {
        this.driveCommandFactory = driveCommandFactory;
        this.driveSubsystem = driveSubsystem;
        this.elevatorCommandFactory = elevatorCommandFactory;
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.highLevelCommandsFactory = highLevelCommandsFactory;
        //registerAllPathPlannerCommands();

        this.autonChooser = new LoggedDashboardChooser<>("AutonChooser");
        addAllAutons();

    }

    

    private void addAllAutons() {
        addAuton(placeCoralBStartingOnOwnAlianceAuton, PlaceCoralBStartingOnOwnAliance.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        addAuton(placeCoralBAndHumanPlayerStartingOnOwnAlianceAuton, PlaceCoralBAndHumanPlayerStartingOnOwnAliance.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        addAuton(placeCoralBAndDStartingOnOwnAlianceAuton, PlaceCoralBAndDStartingOnOwnAliance.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        addAuton(placeCoralBAndDAndHumanPlayerStationStartingOnOwnAlianceAuton, PlaceCoralBAndDAndHumanPlayerStationStartingOnOwnAliance.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        addAuton(placeCoralBAndDAndEStartingOnOwnAlianceAuton, PlaceCoralBAndDAndEStartingOnOwnAliance.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));

        addAuton(placeCoralAStartingCenterBarge, PlaceCoralAStartingCenterBarge.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        addAuton(placeCoralLStartingCenterBarge, PlaceCoralLStartingCenterBarge.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));

        addAuton(placeCoralKStartingOnOtherAlianceAuton, PlaceCoralKStartingOnOtherAliance.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, this.highLevelCommandsFactory));
        addAuton(placeCoralKAndHumanPlayerStartingOnOtherAlianceAuton, PlaceCoralKAndHumanPlayerStartingOnOtherAliance.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, this.highLevelCommandsFactory));
        addAuton(placeCoralKAndIStartingOnOtherAlianceAuton, PlaceCoralKAndIStartingOnOtherAliance.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        addAuton(placeCoralKAndIAndHumanPlayerStationStartingOnOtherAlianceAuton, PlaceCoralKAndIAndHumanPlayerStationStartingOnOtherAliance.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        addAuton(placeCoralKAndIAndHStartingOnOtherAlianceAuton, PlaceCoralKAndIAndHStartingOnOtherAliance.getAutonSupplier(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory, highLevelCommandsFactory));
        
        this.autonChooser.addDefaultOption(placeCoralKAndIAndHStartingOnOtherAlianceAuton,
        placeCoralKAndIAndHStartingOnOtherAlianceAuton);

        // When the sendable chooser value is updated, construct the new autonomous command
        this.autonChooser.getSendableChooser().onChange(new Consumer<String>() {
            @Override
            public void accept(String t) {
                onSelectedAutonChange(t);
            }
        });

        // Initialize the default autonomous
        onSelectedAutonChange(placeCoralKAndIAndHStartingOnOtherAlianceAuton);



    }

    private void addAuton(String autonName, Supplier<Command> autonomousCommand) {
        // Add each auton to the chooser and place each auton supplier in the map
        this.autonChooser.addOption(autonName, autonName);
        autonomousCommands.put(autonName, autonomousCommand);
    }

    private void registerAllPathPlannerCommands() {
        NamedCommands.registerCommand("WaitCommand", new CustomWaitCommand(.2));
    }

    public void onSelectedAutonChange(String newAutonomous) {
        // When selected auton changes, construct the new one
        Logger.recordOutput("Selected Auto", newAutonomous);
        selectedAutonomousCommand = autonomousCommands.getOrDefault(newAutonomous, () -> new Command() {}).get();
    }

    public Command getSelectedAuton() {
        return selectedAutonomousCommand;
    }
}
