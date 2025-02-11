package frc.robot.Auton;

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
import frc.robot.Auton.Autons.TestAutons.ExampleAuton;
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

    public AutonManager(DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem, ElevatorCommandFactory elevatorCommandFactory, OuttakeCommandFactory outtakeCommandFactory) {
        this.driveCommandFactory = driveCommandFactory;
        this.driveSubsystem = driveSubsystem;
        this.elevatorCommandFactory = elevatorCommandFactory;
        this.outtakeCommandFactory = outtakeCommandFactory;
        //registerAllPathPlannerCommands();

        this.autonChooser = new LoggedDashboardChooser<>("AutonChooser");
        addAllAutons();

    }

    private void addAllAutons() {
        addAuton(placeCoralBStartingOnOwnAlianceAuton);
        addAuton(placeCoralBAndHumanPlayerStartingOnOwnAlianceAuton);
        addAuton(placeCoralBAndDStartingOnOwnAlianceAuton);
        addAuton(placeCoralBAndDAndHumanPlayerStationStartingOnOwnAlianceAuton);
        addAuton(placeCoralBAndDAndEStartingOnOwnAlianceAuton);

        addAuton(placeCoralAStartingCenterBarge);
        addAuton(placeCoralLStartingCenterBarge);

        addAuton(placeCoralKStartingOnOtherAlianceAuton);
        addAuton(placeCoralKAndHumanPlayerStartingOnOtherAlianceAuton);
        addAuton(placeCoralKAndIStartingOnOtherAlianceAuton);
        addAuton(placeCoralKAndIAndHumanPlayerStationStartingOnOtherAlianceAuton);
        addAuton(placeCoralKAndIAndHStartingOnOtherAlianceAuton);
        
        this.autonChooser.addDefaultOption(placeCoralKAndIAndHStartingOnOtherAlianceAuton,
        placeCoralKAndIAndHStartingOnOtherAlianceAuton);
    }

    private void addAuton(String autonName) {
        this.autonChooser.addOption(autonName, autonName);
    }

    private void registerAllPathPlannerCommands() {
        NamedCommands.registerCommand("WaitCommand", new CustomWaitCommand(.2));
    }

    public Command getSelectedAuton() {
        // This system allows for auton to be run multable times in one robot init.
        // If this results in a noticable wait before the start of motion this can be swaped out before comp
        Command selectedAuton;
        Logger.recordOutput("Selected Auto", autonChooser.get());
        switch (autonChooser.get()) {
            case placeCoralBStartingOnOwnAlianceAuton:
                selectedAuton = PlaceCoralBStartingOnOwnAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralBAndHumanPlayerStartingOnOwnAlianceAuton:
                selectedAuton = PlaceCoralBAndHumanPlayerStartingOnOwnAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralBAndDStartingOnOwnAlianceAuton:
                selectedAuton = PlaceCoralBAndDStartingOnOwnAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralBAndDAndHumanPlayerStationStartingOnOwnAlianceAuton:
                selectedAuton = PlaceCoralBAndDAndHumanPlayerStationStartingOnOwnAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralBAndDAndEStartingOnOwnAlianceAuton:
                selectedAuton = PlaceCoralBAndDAndEStartingOnOwnAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralLStartingCenterBarge:
                selectedAuton = PlaceCoralLStartingCenterBarge.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralKStartingOnOtherAlianceAuton:
                selectedAuton = PlaceCoralKStartingOnOtherAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralKAndHumanPlayerStartingOnOtherAlianceAuton:
                selectedAuton = PlaceCoralKAndHumanPlayerStartingOnOtherAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralKAndIStartingOnOtherAlianceAuton:
                selectedAuton = PlaceCoralKAndIStartingOnOtherAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralKAndIAndHumanPlayerStationStartingOnOtherAlianceAuton:
                selectedAuton = PlaceCoralKAndIAndHumanPlayerStationStartingOnOtherAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralKAndIAndHStartingOnOtherAlianceAuton:
                selectedAuton = PlaceCoralKAndIAndHStartingOnOtherAliance.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            case placeCoralAStartingCenterBarge:
                selectedAuton = PlaceCoralAStartingCenterBarge.getAuton(driveCommandFactory, driveSubsystem, elevatorCommandFactory, outtakeCommandFactory);
                break;
            default:
                selectedAuton = new Command() {};
                Logger.recordOutput("InvalidAutonName", "InvalidAutonName");
        }
        return selectedAuton;
    }
}
