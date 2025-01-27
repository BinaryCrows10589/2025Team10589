package frc.robot.Subsystems.AlgaeSystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AlgaeCommands.AlgaePivotToPositionCommand;
import frc.robot.Commands.AlgaeCommands.RunAlgaeWheelsCommand;
import frc.robot.Constants.MechanismConstants.AlgaePivotConstants;
import frc.robot.Constants.MechanismConstants.AlgaeWheelConstants;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotSubsystem;
import frc.robot.Subsystems.AlgaeSystem.AlgaeWheels.AlgaeWheelSubsystem;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class AlgaeSystemCommandFactory {
    private final AlgaePivotSubsystem algaePivotSubsystem;
    private final AlgaeWheelSubsystem algaeWheelSubsystem;

    public AlgaeSystemCommandFactory(AlgaePivotSubsystem algaePivotSubsystem, AlgaeWheelSubsystem algaeWheelSubsystem) {
        this.algaePivotSubsystem = algaePivotSubsystem;
        this.algaeWheelSubsystem = algaeWheelSubsystem;
    }

    public RunAlgaeWheelsCommand createAlgaeWheelIntakeCommand() {
        return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kIntakeVoltage);
    }

    public RunAlgaeWheelsCommand createAlgaeWheelOuttakeCommand() {
        return new RunAlgaeWheelsCommand(algaeWheelSubsystem, AlgaeWheelConstants.kOuttakeVoltage);
    }

    public AlgaePivotToPositionCommand createAlgaePivotToIntakePositionCommand() {
        return new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kIntakePositionRotations);
    }

    public AlgaePivotToPositionCommand createAlgaePivotToOuttakePositionCommand() {
        return new AlgaePivotToPositionCommand(algaePivotSubsystem, AlgaePivotConstants.kOuttakePositionRotations);
    }


    public Command createAlgaeIntakeCommand() {
        return new SequentialGroupCommand(
            createAlgaePivotToIntakePositionCommand(),
            createAlgaeWheelIntakeCommand()
        );
    }

    public Command createAlgaeOuttakeCommand() {
        return new SequentialGroupCommand(
            createAlgaePivotToOuttakePositionCommand(),
            createAlgaeWheelOuttakeCommand()
        );
    }

    
}
