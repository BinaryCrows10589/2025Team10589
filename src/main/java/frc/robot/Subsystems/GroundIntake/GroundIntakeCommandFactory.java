package frc.robot.Subsystems.GroundIntake;

import frc.robot.Commands.IntakeCommands.PivotToPositionCommand;
import frc.robot.Commands.IntakeCommands.RunIntakeWheelsCommand;
import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.PivotContants;
import frc.robot.Subsystems.GroundIntake.IntakeWheels.IntakeWheelsSubsystem;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitCoralSensor.TransitCoralSensorSubsystem;

public class GroundIntakeCommandFactory {

    private final PivotSubsystem pivotSubsystem;
    private final IntakeWheelsSubsystem intakeWheelsSubsystem;
    private final TransitCoralSensorSubsystem transitCoralSensorSubsystem;

    public GroundIntakeCommandFactory(PivotSubsystem pivotSubsystem, IntakeWheelsSubsystem intakeWheelsSubsystem, TransitCoralSensorSubsystem transitCoralSensorSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        this.intakeWheelsSubsystem = intakeWheelsSubsystem;
        this.transitCoralSensorSubsystem = transitCoralSensorSubsystem;
    }
     
    public PivotToPositionCommand createPivotDownCommand() {
        return new PivotToPositionCommand(PivotContants.kPivotDownPosition, pivotSubsystem);
    }

    public PivotToPositionCommand createPivotUpCommand() {
        return new PivotToPositionCommand(PivotContants.kPivotUpPosition, pivotSubsystem);
    }

    public RunIntakeWheelsCommand createRunIntakeWheelsCommand() {
        return new RunIntakeWheelsCommand(150, intakeWheelsSubsystem, transitCoralSensorSubsystem);
    }
}
