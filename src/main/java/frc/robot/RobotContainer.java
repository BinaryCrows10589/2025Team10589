// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Auton.AutonManager;
import frc.robot.Commands.SwerveDriveCommands.FieldOrientedDriveCommand;
import frc.robot.Commands.SwerveDriveCommands.LockSwerves;
import frc.robot.Constants.ControlConstants;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.JoystickUtils.ControllerInterface;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually bPe handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {  
    private RobotCreator robotCreator;
    private AutonManager autonManager;

    // Controller Decloration and Instantiation
    private ControllerInterface driverController = new ControllerInterface(ControlConstants.kDriverControllerPort);
    private ControllerInterface buttonBoard = new ControllerInterface(ControlConstants.kButtonBoardPort);
    private ControllerInterface buttonBoardAlt = new ControllerInterface(ControlConstants.kButtonBoardAltPort);
    // Declare all Subsystems and Command Factories
    private DriveSubsystem driveSubsystem;
    private DriveCommandFactory driveCommandFactory;

    // Decloration of Commands
    // SwerveDrive Commands
    private FieldOrientedDriveCommand fieldOrientedDriveCommand;
    private LockSwerves lockSwerves;
    private Command resetOdometry;

    /** 
     * Initalized all Subsystem and Commands 
     */
    public RobotContainer() {
        // Creates all subsystems. Must be first call. 
        this.robotCreator = new RobotCreator();

        this.driveSubsystem = this.robotCreator.getDriveSubsystem();
        this.driveCommandFactory = new DriveCommandFactory(this.driveSubsystem, this.driverController);   
        this.fieldOrientedDriveCommand = this.driveCommandFactory.createFieldOrientedDriveCommand();
        this.driveSubsystem.setDefaultCommand(this.fieldOrientedDriveCommand);
        this.lockSwerves = this.driveCommandFactory.createLockSwervesCommand();
        this.resetOdometry = this.driveCommandFactory.createResetOdometryCommand();
        configureBindings();

        this.autonManager = new AutonManager(this.driveCommandFactory, this.driveSubsystem);
    }

    /**
     * Used to configure button binding
     */
    private void configureBindings() {
        this.driverController.bindToButton(this.lockSwerves, XboxController.Button.kRightBumper.value);
        this.driverController.bindToButton(this.resetOdometry, XboxController.Button.kY.value);
        this.driverController.bindToButton(this.driveCommandFactory.createSwerveDriveTranslationProfiler(), XboxController.Button.kX.value);
        this.driverController.bindToButton(this.driveCommandFactory.createSwerveDriveRotationProfiler(), XboxController.Button.kB.value);
        this.driverController.bindToButton(Commands.runOnce(this.driveSubsystem::setSlowModeTrue),
        Commands.runOnce(this.driveSubsystem::setSlowModeFalse), XboxController.Button.kLeftBumper.value);
    }

    /**
     * Used to select our send our selected autonomus command to the Robot.java file.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {    
        return this.autonManager.getSelectedAuton();
    }

    public void updateAlliance() {
        this.driveSubsystem.getPoseEstimatorSubsystem().updateAlliance();
    }
        
}
