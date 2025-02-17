package frc.robot.Commands.FunnelCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.OuttakeWheelsCommands.HoldCoralInOuttakeCommand;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Utils.LEDUtils.LEDManager;

public class DetectFunnelCoralCommand extends Command {
    
    private final OuttakeCommandFactory outtakeCommandFactory;
    private final HoldCoralInOuttakeCommand holdCoralInOuttakeCommand;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;
    private boolean isSameCoral = false;

    public DetectFunnelCoralCommand(
        OuttakeCommandFactory outtakeCommandFactory,
        OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
        this.holdCoralInOuttakeCommand = this.outtakeCommandFactory.createHoldCoralInOuttakeCommand();
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.isSameCoral = false;
        boolean coralInStartOfOuttake = this.outtakeCoralSensorsSubsystem.isCoralInStartOfOuttake(false);
        if(coralInStartOfOuttake && !isSameCoral) {
            this.holdCoralInOuttakeCommand.schedule();
            this.isSameCoral = true;
        }
        if(!coralInStartOfOuttake && !this.outtakeCoralSensorsSubsystem.isCoralInEndOfOuttake(false)) {
            this.isSameCoral = false;
        }
        Logger.recordOutput("Funnel/HasSeenCoral", this.isSameCoral);
    }

    @Override
    public void end(boolean interrupted) {
        this.isSameCoral = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
