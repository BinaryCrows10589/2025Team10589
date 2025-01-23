package frc.robot.Commands.TransitCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants.TransitConstants;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsIO;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsSubsystem;
import frc.robot.Utils.CommandUtils.Wait;

public class RunTransitToOuttakeCommand extends Command {
    
    private final TransitWheelsSubsystem transitWheelsSubsystem;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;
    private final Wait hardCutOffTimer;

    public RunTransitToOuttakeCommand(double waitTime, TransitWheelsSubsystem transitWheelsSubsystem, OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        this.transitWheelsSubsystem = transitWheelsSubsystem;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
        this.hardCutOffTimer = new Wait(waitTime);
        addRequirements(transitWheelsSubsystem, outtakeCoralSensorsSubsystem);
    }
    public RunTransitToOuttakeCommand(TransitWheelsSubsystem transitWheelsSubsystem, OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        this(150, transitWheelsSubsystem, outtakeCoralSensorsSubsystem);
    }

    @Override
    public void initialize() {
        hardCutOffTimer.startTimer();
        this.transitWheelsSubsystem.setDesiredTransitWheelsVoltage(TransitConstants.TransitWheelsConstants.kTransitWheelsRunningVoltage);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        this.transitWheelsSubsystem.setDesiredTransitWheelsVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return outtakeCoralSensorsSubsystem.isCoralInStartOfOuttake() || hardCutOffTimer.hasTimePassed();
    }
}
