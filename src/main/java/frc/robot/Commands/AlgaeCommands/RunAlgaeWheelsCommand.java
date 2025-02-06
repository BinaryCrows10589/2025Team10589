package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeSystem.AlgaeWheels.AlgaeWheelSubsystem;
import frc.robot.Utils.CommandUtils.Wait;

public class RunAlgaeWheelsCommand extends Command {
    private final AlgaeWheelSubsystem algaeWheelSubsystem;
    private final double desiredVoltage;

    public RunAlgaeWheelsCommand(AlgaeWheelSubsystem algaeWheelSubsystem, double desiredVoltage) {
        this(150, algaeWheelSubsystem, desiredVoltage);
    }

    public RunAlgaeWheelsCommand(double timeout, AlgaeWheelSubsystem algaeWheelSubsystem, double desiredVoltage) {
        this.algaeWheelSubsystem = algaeWheelSubsystem;
        this.desiredVoltage = desiredVoltage;
    }

    @Override
    public void initialize() {
        this.algaeWheelSubsystem.setWheelVoltage(desiredVoltage);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        this.algaeWheelSubsystem.stopWheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
