// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BiConsumer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Auton.AutonManager;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Commands.SwerveDriveCommands.FieldOrientedDriveCommand;
import frc.robot.Commands.SwerveDriveCommands.LockSwerves;
import frc.robot.Constants.GenericConstants.ButtonBoardButtonConstants;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.MechanismConstants.AlgaePivotConstants;
import frc.robot.Subsystems.Climber.ClimberCommandFactory;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.JoystickUtils.ButtonBoardInterface;
import frc.robot.Utils.JoystickUtils.ControllerInterface;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {  
    private final RobotCreator robotCreator;
    private final AutonManager autonManager;

    // Controller Decloration and Instantiation
    private final ControllerInterface driverController = new ControllerInterface(ControlConstants.kDriverControllerPort);
    private final ButtonBoardInterface buttonBoard = new ButtonBoardInterface(ControlConstants.kButtonBoardAutoPositioningPort, ControlConstants.kButtonBoardNormalPort);
    //private final ControllerInterface buttonBoardAlt = new ControllerInterface(ControlConstants.kButtonBoardAltPort);
    // Declare all Subsystems and Command Factories
    private final DriveSubsystem driveSubsystem;
    private final DriveCommandFactory driveCommandFactory;

    private final OuttakeCommandFactory outtakeCommandFactory;
    private final ElevatorCommandFactory elevatorCommandFactory;
    private final ClimberCommandFactory climberCommandFactory;
    //private final TransitWheelsCommandFactory transitWheelsCommandFactory;
    //private final GroundIntakeCommandFactory groundIntakeCommandFactory;

    private final HighLevelCommandsFactory highLevelCommandsFactory;
    // Decloration of Commands
    // SwerveDrive Commands
    private final FieldOrientedDriveCommand fieldOrientedDriveCommand;
    private final LockSwerves lockSwerves;
    private final Command resetOdometry;


    private int basicCommandsRanCount = 0;
    private ArrayList<Command> advancedCommands = new ArrayList<>();

    /** 
     * Initalized all Subsystem and Commands 
     */
    @SuppressWarnings("unused")
    public RobotContainer() {
        // Creates all subsystems. Must be first call. 
        this.robotCreator = new RobotCreator();

        this.driveSubsystem = this.robotCreator.getDriveSubsystem();
        this.driveCommandFactory = new DriveCommandFactory(this.driveSubsystem, this.robotCreator.getElevatorSubsystem(), this.driverController);   
        this.fieldOrientedDriveCommand = this.driveCommandFactory.createFieldOrientedDriveCommand();
        this.driveSubsystem.setDefaultCommand(this.fieldOrientedDriveCommand);
        this.lockSwerves = this.driveCommandFactory.createLockSwervesCommand();
        this.resetOdometry = this.driveCommandFactory.createResetOdometryCommand();
        this.climberCommandFactory = new ClimberCommandFactory(this.robotCreator.getClimberSubsystem());
        
        
        /*this.groundIntakeCommandFactory = new GroundIntakeCommandFactory(this.robotCreator.getPivotSubsystem(),
        this.robotCreator.getIntakeWheelsSubsystem(),
        this.robotCreator.getTransitCoralSensorSubsystem()
        );
        
        this.transitWheelsCommandFactory = new TransitWheelsCommandFactory(this.robotCreator.getTransitWheelsSubsystem(),
            this.robotCreator.getOuttakeCoralSensorsSubsystem());
        */

        this.elevatorCommandFactory = new ElevatorCommandFactory(this.robotCreator.getElevatorSubsystem());


        this.outtakeCommandFactory = new OuttakeCommandFactory(this.robotCreator.getOuttakeWheelsSubsystem(),
            this.robotCreator.getOuttakeCoralSensorsSubsystem());

        this.highLevelCommandsFactory = new HighLevelCommandsFactory(
            this.outtakeCommandFactory, this.robotCreator.getOuttakeCoralSensorsSubsystem(),
             this.robotCreator.getFunnelCoralSensorSubsystem(),
             this.robotCreator.getOuttakeWheelsSubsystem(),
            this.elevatorCommandFactory,
            this.robotCreator.getElevatorSubsystem(),
            this.robotCreator.getPivotSubsystem(),
            this.robotCreator.getAlgaeWheelSubsystem(),
            this.robotCreator.getAlgaePivotSubsystem(),
            this.robotCreator.getReefTreeDetectorSubsystem(),
            this.driveSubsystem
            );
        
        this.robotCreator.getOuttakeWheelsSubsystem().setDefaultCommand(this.highLevelCommandsFactory.createDetectFunnelCoralCommand());

        this.autonManager = new AutonManager(this.driveCommandFactory, this.driveSubsystem, this.elevatorCommandFactory, this.outtakeCommandFactory, this.highLevelCommandsFactory);
        configureBindings();
        if (ControlConstants.basicCommandLogging && !ControlConstants.advancedCommandLogging) configureCommandSchedulerBindings();
        else if (ControlConstants.basicCommandLogging && ControlConstants.advancedCommandLogging) configureAdvancedCommandSchedulerBindings();
        this.onRobotEnable();
    }

    private void logCommand(String commandName, int commandIndex, int commandStateCode, Optional<Command> interruptor) {
        String commandState;
        if (commandStateCode == 0) commandState = "Initialized";
        else if (commandStateCode == 1) commandState = "Executing";
        else if (commandStateCode == 2) commandState = "Finished";
        else commandState = "Interrupted";

        String key = "CommandList/" + commandIndex + "/";

        Logger.recordOutput(key + "Name", commandName);
        Logger.recordOutput(key + "State", commandState);
        if (interruptor != null) {
            Command interruptorOrNull = interruptor.orElse(null);
            if (interruptorOrNull != null)
                Logger.recordOutput(key + "Interruptor", interruptorOrNull.getName());
        }
    }

    private void configureCommandSchedulerBindings() {
        // On command initialize
        CommandScheduler.getInstance().onCommandInitialize(
                (Command command) -> logCommand(command.getName(), basicCommandsRanCount++, 0, null)
        );
    }

    private void advancedCommandLogEvent(Command command, int commandStateCode, Optional<Command> interruptor) {
        int commandIndex = advancedCommands.indexOf(command);
        if (commandIndex == -1) commandIndex = advancedCommands.size();
        logCommand(command.getName(), commandIndex, commandStateCode, interruptor);
    }

    private void configureAdvancedCommandSchedulerBindings() {
        // On command initialize
        CommandScheduler.getInstance().onCommandInitialize(
                (Command command) -> advancedCommandLogEvent(command, 0, null)
        );
        CommandScheduler.getInstance().onCommandExecute(
                (Command command) -> advancedCommandLogEvent(command, 1, null)
        );
        CommandScheduler.getInstance().onCommandFinish(
                (Command command) -> advancedCommandLogEvent(command, 2, null)
        );
        CommandScheduler.getInstance().onCommandInterrupt(
                new BiConsumer<Command, Optional<Command>>() {
                    public void accept(Command command, Optional<Command> interruptor) {
                        advancedCommandLogEvent(command, 3, interruptor);
                    }
                }
        );
    }

    

    /**
     * Used to configure button binding
     */
    private void configureBindings() {
        this.driverController.bindToButton(this.lockSwerves, XboxController.Button.kRightBumper.value);
        this.driverController.bindToButton(this.resetOdometry, XboxController.Button.kY.value);
        //this.driverController.bindToButton(this.driveCommandFactory.createSwerveDriveTranslationProfiler(), XboxController.Button.kX.value);
        //this.driverController.bindToButton(this.driveCommandFactory.createSwerveDriveRotationProfiler(), XboxController.Button.kB.value);
        this.driverController.bindToLeftTriggure(Commands.runOnce(this.driveSubsystem::setSlowModeTrue),
        Commands.runOnce(this.driveSubsystem::setSlowModeFalse));
        /*this.driverController.bindToRightTriggure(Commands.runOnce(this.driveSubsystem::setAxisLockModeTrue),
        Commands.runOnce(this.driveSubsystem::setAxisLockModeFalse));*/
        this.driverController.bindToButton(Commands.runOnce(DriveSubsystem::enableDriverControlleMode), XboxController.Button.kX.value);
         
        this.buttonBoard.bindButton(this.elevatorCommandFactory.createElevatorToBasementCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.l0);
        this.buttonBoard.bindButton(this.highLevelCommandsFactory.createPrepareL1BackfeedCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.l1);
        this.buttonBoard.bindButton(this.elevatorCommandFactory.createElevatorToL2Command(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.l2);
        this.buttonBoard.bindButton(this.elevatorCommandFactory.createElevatorToL3Command(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.l3);
        this.buttonBoard.bindButton(this.elevatorCommandFactory.createElevatorToL4Command(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.l4);

        //this.buttonBoard.bindButton(this.outtakeCommandFactory.createHoldCoralInOuttakeCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.indexCoral);
        this.buttonBoard.bindButton(this.outtakeCommandFactory.createIntakeCoralCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.outtakeIn);
        this.buttonBoard.bindButton(this.outtakeCommandFactory.createOuttakeCoralCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.outtakeOut);
        this.buttonBoard.bindButton(this.outtakeCommandFactory.createBackfeedCoralCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.l1Backfeed);

        this.buttonBoard.bindButton(highLevelCommandsFactory.createIntakeAlgaeFromGroundCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.groundIntake);
        this.buttonBoard.bindButton(highLevelCommandsFactory.createIntakeAlgaeFromReefL2Command(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.reefIntakeLow);
        this.buttonBoard.bindButton(highLevelCommandsFactory.createIntakeAlgaeFromReefL3Command(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.reefIntakeHigh);
        this.buttonBoard.bindButton(this.highLevelCommandsFactory.createLollypopIntakeCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.intakeLolypop);

        this.buttonBoard.bindButton(this.highLevelCommandsFactory.createOuttakeAlgaeInProcessorCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.processorOuttakeAlgae);
        this.buttonBoard.bindButton(this.highLevelCommandsFactory.createOuttakeAlgaeOnBargeCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.bargeOuttakeAlgae);
        this.buttonBoard.bindButton(this.highLevelCommandsFactory.createOutakeWheelsAlgaeProcessorCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.processorOuttakeAlgaeWheels);
        this.buttonBoard.bindButton(this.highLevelCommandsFactory.createOutakeWheelsAlgaeBargeCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.bargeOuttakeAlgaeWheels);

        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralLeftCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeOne);
        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralRightCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeTwo);
        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralLeftCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeThree);
        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralRightCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeFour);
        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralLeftCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeFive);
        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralRightCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeSix);

        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralLeftCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeSeven);
        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralRightCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeEight);
        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralLeftCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeNine);
        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralRightCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeTen);
        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralLeftCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeEleven);
        this.buttonBoard.bindAutoPositioningCommand(this.highLevelCommandsFactory.createPlaceCoralRightCommand(), ButtonBoardButtonConstants.ButtonBoardAutoPositioningButtons.reefTreeTwelve);
        
        //this.buttonBoard.bindButton(this.elevatorCommandFactory.createMoveElevatorUpCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.elevatorUp);
        this.buttonBoard.bindButton(Commands.runOnce(DriveSubsystem::disableDriverControlleMode), Commands.runOnce(DriveSubsystem::enableDriverControlleMode), ButtonBoardButtonConstants.ButtonBoardNormalButtons.disableDriving);
        
        //this.buttonBoard.bindButton(this.climberCommandFactory.createpPivotClimberDownCommand(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.climberDown);
        //this.buttonBoard.bindButton(this.climberCommandFactory.createPivotClimberUpCommnad(), ButtonBoardButtonConstants.ButtonBoardNormalButtons.climberUp);
        /*      
        this.buttonBoardAlt.bindToButton(this.outtakeCommandFactory.createOuttakeCoralCommand(), XboxController.Button.kA.value);
        this.buttonBoardAlt.bindToButton(this.outtakeCommandFactory.createHoldCoralInOuttakeCommand(), XboxController.Button.kB.value);
        
        this.buttonBoardAlt.bindToPOV(this.highLevelCommandsFactory.createOutakeWheelsAlgaeBargeCommand(), 0);
        this.buttonBoardAlt.bindToPOV(this.highLevelCommandsFactory.createOutakeWheelsAlgaeProcessorCommand(), 90);
        this.buttonBoardAlt.bindToPOV(this.highLevelCommandsFactory.createAlgaeWheelsIntakeGroundCommand(), 180);
        this.buttonBoardAlt.bindToPOV(this.highLevelCommandsFactory.createAlgaeWheelsIntakeReefTreeCommand(), 270);

        //this.buttonBoardAlt.bindToButton(this.highLevelCommandsFactory.createPlaceCoralRightCommand(), XboxController.Button.kX.value);
        //this.buttonBoardAlt.bindToButton(this.highLevelCommandsFactory.createPlaceCoralLeftCommand(), XboxController.Button.kY.value);
         this.buttonBoardAlt.bindToButton(new AlgaePivotToPositionCommand(this.robotCreator.getAlgaePivotSubsystem(), AlgaePivotConstants.kDefultPivotPosition), XboxController.Button.kX.value);
         this.buttonBoardAlt.bindToButton(new AlgaePivotToPositionCommand(this.robotCreator.getAlgaePivotSubsystem(), AlgaePivotConstants.kGroundIntakePositionRotations), XboxController.Button.kY.value);

        this.buttonBoardAlt.bindToButton(this.elevatorCommandFactory.createElevatorToBasementCommand(), XboxController.Button.kLeftStick.value);  
        this.buttonBoardAlt.bindToButton(this.elevatorCommandFactory.createElevatorToL2Command(), XboxController.Button.kLeftBumper.value); 
        this.buttonBoardAlt.bindToLeftTriggure(this.elevatorCommandFactory.createElevatorToL3Command());  
        this.buttonBoardAlt.bindToRightTriggure(this.elevatorCommandFactory.createElevatorToL4Command());  
        */
    }

    /**
     * Used to select our send our selected autonomus command to the Robot.java file.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {   
       // this.driveSubsystem.setRobotPose(new AutonPoint(5.5, 5.5, 50));
      // this.placeCommand;//
        return this.autonManager.getSelectedAuton();
    }

    public Pose2d getRobotPosition() {
        return driveSubsystem.getRobotPose();
    }

    public void updateAlliance() {
        this.driveSubsystem.getPoseEstimatorSubsystem().updateAlliance();
    }

    public void updateButtonBoardInputs() {
        this.buttonBoard.periodic();
        
    }

    public void periodic() {
        ControlConstants.kHasCoral = this.robotCreator.getOuttakeCoralSensorsSubsystem().isCoralInEndOfOuttake(false);
        Logger.recordOutput("HasCoral/HasCoral", ControlConstants.kHasCoral);

        /*if(DriverStation.isDisabled()) {
            Pose2d robotPose = this.driveSubsystem.getRobotPose();
            Pose2d startingPose = ControlConstants.robotStartPosition.getAutonPoint();
            boolean translationXInTolorence = Tolerance.inTolorance(robotPose.getX(), startingPose.getX(),
                ControlConstants.robotStartPositionTolorence.getX());
            boolean translationYInTolorence = Tolerance.inTolorance(robotPose.getY(), startingPose.getY(),
                ControlConstants.robotStartPositionTolorence.getY());
            boolean rotationalInTolorence = Tolerance.inToloranceRotation(
                robotPose.getRotation(), startingPose.getRotation(),
                ControlConstants.robotStartPositionTolorence.getRotation().getRadians());
            if(translationXInTolorence && translationYInTolorence && rotationalInTolorence) {
                LEDManager.setSolidColor(new int[] {0, 255, 0});
            } else {
                LEDManager.setSolidColor(new int[] {255, 0, 0});
            }
        }*/
        
    }

    public void onRobotEnable() {
        this.robotCreator.getAlgaePivotSubsystem().setDesiredPivotRotation(AlgaePivotConstants.kDefultPivotPosition);
        this.robotCreator.getElevatorSubsystem().disableElevatorMotors();
    }
        
}
