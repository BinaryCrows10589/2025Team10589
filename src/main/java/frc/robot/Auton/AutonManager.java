package frc.robot.Auton;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.Autons.RealRobotTestAuton;
import frc.robot.Auton.Autons.DriveToGameObjectTestAuton;
import frc.robot.Auton.Autons.ExampleAuton;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.CommandUtils.CustomWaitCommand;

public class AutonManager {
    // Decloration of auton names
    private final String exampleAutonName = "ExampleAuton";
    private final String realRobotTestAutonName = "RealRobotTestAuton";
    private final String driveToGameObjectTestAutonName = "DriveToGameObjectTestAuton";

    // Decloration of auton chooser
    private LoggedDashboardChooser<String> autonChooser;

    // Decloration of all auton dependencies
    private AutonPointManager autonPointManager;
    private DriveCommandFactory driveCommandFactory;
    private DriveSubsystem driveSubsystem;

    public AutonManager(DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        this.autonPointManager = new AutonPointManager();
        this.driveCommandFactory = driveCommandFactory;
        this.driveSubsystem = driveSubsystem;
        registerAllPathPlannerCommands();

        this.autonChooser = new LoggedDashboardChooser<>("AutonChooser");
        addAllAutons();

    }

    private void addAllAutons() {
        addAuton(exampleAutonName);
        addAuton(realRobotTestAutonName);
        addAuton(driveToGameObjectTestAutonName);
        this.autonChooser.addDefaultOption(realRobotTestAutonName, realRobotTestAutonName);
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
            case exampleAutonName:
                selectedAuton = ExampleAuton.getAuton(this.autonPointManager, this.driveCommandFactory, this.driveSubsystem);
                break;
            case realRobotTestAutonName:
                selectedAuton = RealRobotTestAuton.getAuton(this.autonPointManager, this.driveCommandFactory, this.driveSubsystem);
                break;
            case driveToGameObjectTestAutonName:
                selectedAuton = DriveToGameObjectTestAuton.getAuton(this.autonPointManager, this.driveCommandFactory, this.driveSubsystem);
                break;
            default:
                selectedAuton = new Command() {};
                Logger.recordOutput("InvalidAutonName", "InvalidAutonName");
        }
        return selectedAuton;
    }
}
