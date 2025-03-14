package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommands.ElevatorToPositionCommand;
import frc.robot.Commands.OuttakeWheelsCommands.HoldCoralInOuttakeCommand;
import frc.robot.Commands.TransitCommands.RunTransitToOuttakeCommand;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.GroundIntake.GroundIntakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitCoralSensor.TransitCoralSensorSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsCommandFactory;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;
import frc.robot.Utils.LEDUtils.LEDManager;

public class GroundIntakeCommand extends Command {

    /* This command needs to:
     * - Lower intake whilst running intake wheels
     * - Bring the elevator to the basement
     * - Start running the transit wheels WHEN ELEVATOR REACHES BASEMENT (check for command ending)
     * - Wait for coral to appear in transit
     * - Wait for coral to appear in outtake
     * - Hold coral
     * - Reset everything
     */

    public static enum GroundIntakeCommandStage {
        INTAKING,
        RESETTING
    }

    private GroundIntakeCommandStage currentState = GroundIntakeCommandStage.INTAKING;
    private boolean hasStartedNextStage = false;
    private final GroundIntakeCommandFactory groundIntakeCommandFactory;
    private final TransitWheelsCommandFactory transitWheelsCommandFactory;
    private final OuttakeCommandFactory outtakeCommandFactory;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;
    private final TransitCoralSensorSubsystem transitCoralSensorSubsystem;
    private final ElevatorCommandFactory elevatorCommandFactory;
    private final PivotToPositionCommand pivotUpCommand;
    private final PivotToPositionCommand pivotDownCommand;
    private final ElevatorToPositionCommand elevatorToL1Command;
    private final ElevatorToPositionCommand elevatorToBasementCommand;
    private final HoldCoralInOuttakeCommand holdCoralInOuttakeCommand;
    private final RunTransitToOuttakeCommand runTransitToOuttakeCommand;
    private final RunIntakeWheelsCommand runIntakeWheelsCommand;
            
    private SequentialGroupCommand elevatorAndTransitCommand;
    

    public GroundIntakeCommand(
        GroundIntakeCommandFactory groundIntakeCommandFactory, 
        TransitWheelsCommandFactory transitWheelsCommandFactory, 
        OuttakeCommandFactory outtakeCommandFactory, 
        OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem,
        TransitCoralSensorSubsystem transitCoralSensorSubsystem,
        ElevatorCommandFactory elevatorCommandFactory) {
            this.groundIntakeCommandFactory = groundIntakeCommandFactory;
            this.transitWheelsCommandFactory = transitWheelsCommandFactory;
            this.outtakeCommandFactory = outtakeCommandFactory;
            this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
            this.transitCoralSensorSubsystem = transitCoralSensorSubsystem;
            this.elevatorCommandFactory = elevatorCommandFactory;
            this.pivotUpCommand = this.groundIntakeCommandFactory.createPivotUpCommand();
            this.elevatorToL1Command = this.elevatorCommandFactory.createElevatorToL1Command();
            this.holdCoralInOuttakeCommand = this.outtakeCommandFactory.createHoldCoralInOuttakeCommandWithWaitTime(.5);
            this.runTransitToOuttakeCommand = this.transitWheelsCommandFactory.createRunTransitToOuttakeCommand();
            this.pivotDownCommand = this.groundIntakeCommandFactory.createPivotDownCommand();
            this.elevatorToBasementCommand = this.elevatorCommandFactory.createElevatorToBasementCommand();
            this.runIntakeWheelsCommand = this.groundIntakeCommandFactory.createRunIntakeWheelsCommand();
    }

    @Override
    public void initialize() {
        elevatorAndTransitCommand = new SequentialGroupCommand(
            this.elevatorToBasementCommand, 
            this.runTransitToOuttakeCommand);        
        //LEDManager.setSolidColor(ControlConstants.kCoralIntakingColor);

    }

    public boolean checkIfStageStarted() {
        if (hasStartedNextStage) { // If we've started, say so
            return true;
        } // Otherwise...
        // Flip hasStartedNextStage so that this method only returns "false" once until we goToNextStage again
        hasStartedNextStage = true;
        return false;
    }
    public void goToNextStage(GroundIntakeCommandStage stage) {
        currentState = stage;
        hasStartedNextStage = false;
    }
    
    @Override
    public void execute() {
        if (currentState == GroundIntakeCommandStage.INTAKING) {
            if (!checkIfStageStarted()) {
                this.pivotDownCommand.schedule(); // Moves pivot down, stops when it's there
                this.runIntakeWheelsCommand.schedule(); // Starts pivot wheels, stops when coral is in the transit
                this.elevatorAndTransitCommand.schedule();
            }
            if (elevatorAndTransitCommand.isFinished()) { // True when coral is in the outtake
                goToNextStage(GroundIntakeCommandStage.RESETTING);
            }
        }
    }     
    @Override
    public void end(boolean interrupted) {
        this.currentState = GroundIntakeCommandStage.INTAKING;
        this.hasStartedNextStage = false;
        this.elevatorAndTransitCommand.cancel();
        this.elevatorToBasementCommand.cancel();
        this.pivotDownCommand.cancel();
        this.pivotUpCommand.schedule(); // Pivots the pivot back into the robot
        this.elevatorToL1Command.schedule();// Brings elevator up to L1
        this.holdCoralInOuttakeCommand.schedule();
        LEDManager.setSolidColor(ControlConstants.kCoralIntakedColor);
    }

    @Override
    public boolean isFinished() {
        return currentState == GroundIntakeCommandStage.RESETTING;// True when we have set everything up to return to where it needs to be
    }


}
