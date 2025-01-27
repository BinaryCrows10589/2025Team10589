package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Elevator.ElevatorCommandFactory;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.GroundIntake.GroundIntakeCommandFactory;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsSubsystem;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitCoralSensor.TransitCoralSensorSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsCommandFactory;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

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
            addRequirements(outtakeCoralSensorsSubsystem, transitCoralSensorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorAndTransitCommand = new SequentialGroupCommand(
            elevatorCommandFactory.createElevatorToBasementCommand(),
            transitWheelsCommandFactory.createRunTransitToOuttakeCommand());
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
                groundIntakeCommandFactory.createPivotDownCommand().schedule(); // Moves pivot down, stops when it's there
                groundIntakeCommandFactory.createRunIntakeWheelsCommand().schedule(); // Starts pivot wheels, stops when coral is in the transit
                elevatorAndTransitCommand.schedule();
            }
            if (elevatorAndTransitCommand.isFinished()) { // True when coral is in the outtake
                goToNextStage(GroundIntakeCommandStage.RESETTING);
            }
        } else if (currentState == GroundIntakeCommandStage.RESETTING) {
            if (!checkIfStageStarted()) {
                groundIntakeCommandFactory.createPivotUpCommand().schedule(); // Pivots the pivot back into the robot
                elevatorCommandFactory.createElevatorToL1Command().schedule(); // Brings elevator up to L1
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return currentState == GroundIntakeCommandStage.RESETTING && hasStartedNextStage; // True when we have set everything up to return to where it needs to be
    }


}
