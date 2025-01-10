package frc.robot.Auton;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.Autons.TestAutons.ExampleAuton;
import frc.robot.Auton.Autons.TestAutons.OwnAlianceBargeStartAutons.PlaceCoralBAndHumanPlayerStartingOnOwnAliance;
import frc.robot.Auton.Autons.TestAutons.OwnAlianceBargeStartAutons.PlaceCoralBStartingOnOwnAliance;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.CommandUtils.CustomWaitCommand;

public class AutonManager {
    // Decloration of auton names
    private final String placeCoralBStartingOnOwnAlianceAuton = "PlaceCoralBStartingOnOwnAliance";
    private final String placeCoralBAndHumanPlayerStartingOnOwnAlianceAuton = "PlaceCoralBAndHumanPlayerStartingOnOwnAliance";

    // Decloration of auton chooser
    private LoggedDashboardChooser<String> autonChooser;

    // Decloration of all auton dependencies
    private DriveCommandFactory driveCommandFactory;
    private DriveSubsystem driveSubsystem;

    public AutonManager(DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        this.driveCommandFactory = driveCommandFactory;
        this.driveSubsystem = driveSubsystem;
        //registerAllPathPlannerCommands();

        this.autonChooser = new LoggedDashboardChooser<>("AutonChooser");
        addAllAutons();

    }

    private void addAllAutons() {
        addAuton(placeCoralBStartingOnOwnAlianceAuton);
        addAuton(placeCoralBAndHumanPlayerStartingOnOwnAlianceAuton);
        this.autonChooser.addDefaultOption(placeCoralBStartingOnOwnAlianceAuton, placeCoralBStartingOnOwnAlianceAuton);
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
                selectedAuton = PlaceCoralBStartingOnOwnAliance.getAuton(driveCommandFactory, driveSubsystem);
                break;
            case placeCoralBAndHumanPlayerStartingOnOwnAlianceAuton:
                selectedAuton = PlaceCoralBAndHumanPlayerStartingOnOwnAliance.getAuton(driveCommandFactory, driveSubsystem);
                break;
            default:
                selectedAuton = new Command() {};
                Logger.recordOutput("InvalidAutonName", "InvalidAutonName");
        }
        return selectedAuton;
    }
}
