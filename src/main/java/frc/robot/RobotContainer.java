// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Auton.AutonManager;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Commands.SwerveDriveCommands.FieldOrientedDriveCommand;
import frc.robot.Commands.SwerveDriveCommands.LockSwerves;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.GroundIntake.GroundIntakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsCommandFactory;
import frc.robot.Utils.JoystickUtils.ButtonBoardInterface;
import frc.robot.Utils.JoystickUtils.ControllerInterface;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually bPe handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {  
    private final RobotCreator robotCreator;
    private final AutonManager autonManager;

    // Controller Decloration and Instantiation
    private final ControllerInterface driverController = new ControllerInterface(ControlConstants.kDriverControllerPort);
    private final ButtonBoardInterface buttonBoard = new ButtonBoardInterface(ControlConstants.kButtonBoardPort);
    private final ControllerInterface buttonBoardAlt = new ControllerInterface(ControlConstants.kButtonBoardAltPort);
    // Declare all Subsystems and Command Factories
    private final DriveSubsystem driveSubsystem;
    private final DriveCommandFactory driveCommandFactory;

    private final OuttakeCommandFactory outtakeCommandFactory;
    private final ElevatorCommandFactory elevatorCommandFactory;
    private final TransitWheelsCommandFactory transitWheelsCommandFactory;

    private final HighLevelCommandsFactory highLevelCommandsFactory;
    // Decloration of Commands
    // SwerveDrive Commands
    private final FieldOrientedDriveCommand fieldOrientedDriveCommand;
    private final LockSwerves lockSwerves;
    private final Command resetOdometry;

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

        this.outtakeCommandFactory = new OuttakeCommandFactory(this.robotCreator.getOuttakeWheelsSubsystem(),
            this.robotCreator.getOuttakeCoralSensorsSubsystem());
        this.elevatorCommandFactory = new ElevatorCommandFactory(this.robotCreator.getElevatorSubsystem());
        this.transitWheelsCommandFactory = new TransitWheelsCommandFactory(this.robotCreator.getTransitWheelsSubsystem(),
            this.robotCreator.getOuttakeCoralSensorsSubsystem());


        this.highLevelCommandsFactory = new HighLevelCommandsFactory(new GroundIntakeCommandFactory(this.robotCreator.getPivotSubsystem(),
            this.robotCreator.getIntakeWheelsSubsystem(),
            this.robotCreator.getTransitCoralSensorSubsystem()
            ),
            this.transitWheelsCommandFactory, this.outtakeCommandFactory, this.robotCreator.getOuttakeCoralSensorsSubsystem(),
            this.robotCreator.getTransitCoralSensorSubsystem(), this.robotCreator.getFunnelCoralSensorSubsystem(),
            this.elevatorCommandFactory);

        //this.robotCreator.getFunnelCoralSensorSubsystem().setDefaultCommand(this.highLevelCommandsFactory.createDetectFunnelCoralCommand());
        configureBindings();

        this.autonManager = new AutonManager(this.driveCommandFactory, this.driveSubsystem);
    }

    /**
     * Used to configure button binding
     */
    private void configureBindings() {
        this.driverController.bindToButton(this.lockSwerves, XboxController.Button.kLeftBumper.value);
        this.driverController.bindToButton(this.resetOdometry, XboxController.Button.kY.value);
        this.driverController.bindToButton(this.driveCommandFactory.createSwerveDriveTranslationProfiler(), XboxController.Button.kX.value);
        this.driverController.bindToButton(this.driveCommandFactory.createSwerveDriveRotationProfiler(), XboxController.Button.kB.value);
        this.driverController.bindToLeftTriggure(Commands.runOnce(this.driveSubsystem::setSlowModeTrue),
        Commands.runOnce(this.driveSubsystem::setSlowModeFalse));
        this.buttonBoardAlt.bindToButton(this.outtakeCommandFactory.createOuttakeCoralCommand(), XboxController.Button.kA.value);
        this.buttonBoardAlt.bindToButton(this.highLevelCommandsFactory.createGroundIntakeCommand(), XboxController.Button.kB.value);
        
        //t(outtakeCommandFactory.createOuttakeCoralCommand(), XboxController.Button.kA.value);
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

    public void updateButtonBoardInputs() {
        this.buttonBoard.periodic();
        
    }
        
}
