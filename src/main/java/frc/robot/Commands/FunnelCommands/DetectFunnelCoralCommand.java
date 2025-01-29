package frc.robot.Commands.FunnelCommands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.jni.ControlConfigJNI;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.OuttakeWheelsCommands.HoldCoralInOuttakeCommand;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.MechanismConstants.TransitConstants;
import frc.robot.Subsystems.Funnel.FunnelCoralSensor.FunnelCoralSensorSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsIO;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsSubsystem;
import frc.robot.Utils.CommandUtils.Wait;
import frc.robot.Utils.LEDUtils.LEDManager;

public class DetectFunnelCoralCommand extends Command {
    
    private final FunnelCoralSensorSubsystem funnelCoralSensorSubsystem;
    private final OuttakeCommandFactory outtakeCommandFactory;
    private final HoldCoralInOuttakeCommand holdCoralInOuttakeCommand;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;
    private boolean isSameCoral = false;

    public DetectFunnelCoralCommand(
        OuttakeCommandFactory outtakeCommandFactory,
        OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem, FunnelCoralSensorSubsystem funnelCoralSensorSubsystem) {
        this.funnelCoralSensorSubsystem = funnelCoralSensorSubsystem;
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
        this.holdCoralInOuttakeCommand = this.outtakeCommandFactory.createHoldCoralInOuttakeCommand();

        addRequirements(funnelCoralSensorSubsystem);
    }


    @Override
    public void initialize() {
       this.isSameCoral = false;
    }

    @Override
    public void execute() {
        if(this.funnelCoralSensorSubsystem.isCoralInFunnel() && !isSameCoral) {
            LEDManager.setSolidColor(ControlConstants.kCoralIntakingColor);
            this.holdCoralInOuttakeCommand.cancel();
            this.isSameCoral = true;
        } else if(this.outtakeCoralSensorsSubsystem.isCoralInStartOfOuttake(false) && isSameCoral) {
            LEDManager.setSolidColor(ControlConstants.kCoralIntakedColor);
            this.isSameCoral = false;
            this.holdCoralInOuttakeCommand.schedule();
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
